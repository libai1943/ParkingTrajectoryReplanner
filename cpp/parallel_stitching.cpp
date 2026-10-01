// Chapter 7: parallel connection search and three-segment trajectory assembly.
// Chinese book: 非结构化场景自动驾驶轨迹规划技术.
// Cite Li et al., IEEE T-IV 7(3):748-757, 2022. DOI:10.1109/TIV.2022.3156429.
// Copyright 2022-2026 Bai Li. PolyForm Noncommercial 1.0.0, see ../LICENSE.
#include "parking_backend.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <thread>

using Clock = std::chrono::steady_clock;
struct Task { int i, j; double start_time, end_time; State start, finish; };
struct Validation { double join = 0, dynamics = 0, bounds = 0; bool collision_free = false, increasing = true; };
struct Process { PROCESS_INFORMATION info{}; fs::path output; int task = -1; };

State sample(const Trajectory& trajectory, double time) {
  auto right = std::upper_bound(trajectory.begin(), trajectory.end(), time,
      [](double value, const Sample& row) { return value < row[0]; });
  State result{};
  if (right == trajectory.begin()) { std::copy_n(right->begin()+1, 7, result.begin()); return result; }
  if (right == trajectory.end()) { std::copy_n(trajectory.back().begin()+1, 7, result.begin()); return result; }
  const auto& left = *(right-1);
  double fraction = (time-left[0])/((*right)[0]-left[0]);
  for (int j=0; j<7; ++j) result[j] = left[j+1] + fraction*((*right)[j+1]-left[j+1]);
  return result;
}

std::vector<Task> build_tasks(const Trajectory& original, const Trajectory& evasive, double t0, double tbrake) {
  std::vector<Task> tasks;
  for (int i=0; i<5; ++i) {
    double start_time = t0+1.2+(tbrake-t0-1.2)*i/4;
    State start = sample(original, start_time);
    auto nearest = std::min_element(evasive.begin(), evasive.end(), [&](const Sample& a, const Sample& b) {
      return std::hypot(a[1]-start[0],a[2]-start[1]) < std::hypot(b[1]-start[0],b[2]-start[1]);
    });
    double duration = evasive.back()[0];
    double lower = std::min((*nearest)[0]+.05*duration, duration);
    double upper = std::min((*nearest)[0]+.30*duration, duration);
    for (int j=0; j<6; ++j) {
      double end_time = lower+(upper-lower)*j/5;
      State finish = sample(evasive, end_time);
      finish[2] = start[2]+std::atan2(std::sin(finish[2]-start[2]),std::cos(finish[2]-start[2]));
      tasks.push_back({i+1,j+1,start_time,end_time,start,finish});
    }
  }
  return tasks;
}

Trajectory assemble(const Trajectory& original, const Trajectory& evasive,
                    const Task& task, const Trajectory& connector, double t0) {
  Trajectory result;
  State first = sample(original,t0); Sample row{}; row[0]=t0;
  std::copy(first.begin(),first.end(),row.begin()+1); result.push_back(row);
  for (auto p: original) if (p[0]>t0 && p[0]<task.start_time) result.push_back(p);
  for (auto p: connector) { p[0]+=task.start_time; result.push_back(p); }
  double angle_shift = task.finish[2]-sample(evasive,task.end_time)[2];
  for (auto p: evasive) if (p[0]>task.end_time) {
    p[0]+=task.start_time+connector.back()[0]-task.end_time;
    p[3]+=angle_shift; result.push_back(p);
  }
  return result;
}

Validation validate(const ParkingBackend& backend, const Trajectory& full,
                    const Task& task, const Trajectory& connector) {
  Validation v;
  for (const auto& row : full) for (double x : row)
    if (!std::isfinite(x)) { v.join=std::numeric_limits<double>::infinity(); return v; }
  for (int j=0;j<7;++j) v.join=std::max({v.join,std::abs(connector.front()[j+1]-task.start[j]),
                                                     std::abs(connector.back()[j+1]-task.finish[j])});
  for (size_t i=0;i<connector.size();++i) {
    auto z=connector[i]; const std::array<double,4> limits{3,2,.85,.7};
    for(int j=0;j<4;++j) v.bounds=std::max(v.bounds,std::abs(z[j+4])-limits[j]);
    if(i+1==connector.size()) continue;
    double dt=connector[i+1][0]-z[0];
    const std::array<int,5> columns{1,2,3,4,6};
    const std::array<double,5> rhs{z[4]*cos(z[3]),z[4]*sin(z[3]),z[4]*tan(z[6])/2.8,z[5],z[7]};
    for(int j=0;j<5;++j) v.dynamics=std::max(v.dynamics,std::abs(connector[i+1][columns[j]]-z[columns[j]]-dt*rhs[j]));
  }
  for(size_t i=1;i<full.size();++i) v.increasing=v.increasing && full[i][0]>full[i-1][0];
  v.collision_free=backend.collision_free(full);
  return v;
}

Trajectory brake(const Trajectory& original, double t0, double time) {
  Trajectory result; for(auto p:original) if(p[0]>=t0 && p[0]<time) result.push_back(p);
  State z=sample(original,time);
  auto append=[&] { Sample row{};row[0]=time;std::copy(z.begin(),z.end(),row.begin()+1);result.push_back(row); };
  append();
  while(std::abs(z[3])>1e-10) {
    double dt=std::min(.01,std::abs(z[3])/2),a=z[3]>0?-2.:2.;
    z[0]+=dt*z[3]*cos(z[2]);z[1]+=dt*z[3]*sin(z[2]);z[2]+=dt*z[3]*tan(z[5])/2.8;
    z[3]+=dt*a;z[4]=a;z[6]=0;time+=dt;append();
  }
  result.back()[5]=0;return result;
}

void write_csv(const fs::path& path,const Trajectory& trajectory) {
  std::ofstream out(path); if(!out)throw std::runtime_error("Cannot write trajectory output.");
  out<<std::setprecision(17);
  for(auto row:trajectory) { for(int j=0;j<8;++j)out<<(j?",":"")<<row[j];out<<'\n'; }
}
Trajectory read_csv(const fs::path& path) {
  std::ifstream input(path);Trajectory result;std::string line;
  while(std::getline(input,line)){std::replace(line.begin(),line.end(),',',' ');std::istringstream stream(line);Sample row{};
    for(auto& value:row)if(!(stream>>value))throw std::runtime_error("Invalid worker trajectory.");result.push_back(row);}
  if(result.empty())throw std::runtime_error("Worker produced no trajectory.");return result;
}
std::wstring quote(const std::wstring& value) { return L"\""+value+L"\""; }
Process launch(const fs::path& executable,const std::vector<std::wstring>& arguments,const fs::path& output,int task) {
  if(fs::exists(output))fs::remove(output);
  std::wstring command=quote(executable.wstring());for(auto& argument:arguments)command+=L" "+quote(argument);
  STARTUPINFOW startup{};startup.cb=sizeof startup;Process process;process.output=output;process.task=task;
  if(!CreateProcessW(executable.c_str(),command.data(),nullptr,nullptr,FALSE,CREATE_NO_WINDOW,nullptr,nullptr,&startup,&process.info))
    throw std::runtime_error("Cannot start independent solver worker.");
  return process;
}
bool finished(const Process& process) {return WaitForSingleObject(process.info.hProcess,0)==WAIT_OBJECT_0;}
void close_process(Process& process,bool terminate=false) {
  if(terminate && !finished(process))TerminateProcess(process.info.hProcess,2);
  WaitForSingleObject(process.info.hProcess,INFINITE);
  CloseHandle(process.info.hThread);CloseHandle(process.info.hProcess);
}

int wmain(int argc,wchar_t** argv) {
  try {
    fs::path root=fs::path(PARKING_SOURCE_ROOT),runtime,output;
    int case_id=1,workers=4;bool enforce_deadline=false,evasive_worker=false;fs::path boundary_file,worker_output;
    for(int i=1;i<argc;++i){std::wstring arg=argv[i];auto next=[&](){if(i+1>=argc)throw std::runtime_error("Missing option value.");return std::wstring(argv[++i]);};
      if(arg==L"--root")root=next();else if(arg==L"--runtime")runtime=next();else if(arg==L"--case")case_id=std::stoi(next());
      else if(arg==L"--workers")workers=std::stoi(next());else if(arg==L"--output")output=next();else if(arg==L"--deadline")enforce_deadline=true;
      else if(arg==L"--evasive-worker")evasive_worker=true;else if(arg==L"--boundary")boundary_file=next();else if(arg==L"--worker-output")worker_output=next();
      else throw std::runtime_error("Unknown option. See README.");}
    if(runtime.empty())throw std::runtime_error("Provide --runtime pointing to the CasADi 3.7.2 package directory.");
    if(workers<1||workers>32)throw std::runtime_error("workers must be between 1 and 32.");
    ParkingBackend backend(root,runtime,case_id);
    if(evasive_worker){write_csv(worker_output,backend.evasive());return 0;}
    if(!boundary_file.empty()) {
      std::ifstream input(boundary_file);State start{},finish{};for(auto& x:start)input>>x;for(auto& x:finish)input>>x;
      if(!input)throw std::runtime_error("Invalid worker boundary values.");
      write_csv(worker_output,backend.connect(start,finish));return 0;
    }
    if(output.empty())output=root/"cpp/results";output=fs::absolute(output);fs::create_directories(output/"workers");
    wchar_t program[32768];GetModuleFileNameW(nullptr,program,32768);fs::path executable=program;
    const auto original=backend.original();const auto [t0,t1,tbrake,tf]=backend.metadata;
    auto started=Clock::now();auto elapsed=[&](){return std::chrono::duration<double>(Clock::now()-started).count();};
    Trajectory selected,evasive;Validation validation;double best_cost=std::numeric_limits<double>::infinity();
    int selected_i=0,selected_j=0,completed=0,valid_count=0;std::string status;
    const std::vector<std::wstring> common{L"--root",root.wstring(),L"--runtime",runtime.wstring(),L"--case",std::to_wstring(case_id)};
    if(tbrake<t0+1.2){status="fail_safe_insufficient_window";selected=brake(original,t0,t0);}
    else {
      auto file=output/"workers/evasive.csv";auto arguments=common;
      arguments.insert(arguments.end(),{L"--evasive-worker",L"--worker-output",file.wstring()});
      Process process=launch(executable,arguments,file,-1);
      while(!finished(process) && !(enforce_deadline && elapsed()>=1.2))std::this_thread::sleep_for(std::chrono::milliseconds(2));
      bool late=enforce_deadline && elapsed()>=1.2;close_process(process,late);
      if(!late && fs::exists(file))evasive=read_csv(file);
      if(evasive.empty())status="fail_safe_evasion_or_deadline";
      else {
        const auto tasks=build_tasks(original,evasive,t0,tbrake);size_t next=0;std::vector<Process> active;
        while(next<tasks.size()||!active.empty()) {
          if(enforce_deadline && elapsed()>=1.2){for(auto& p:active)close_process(p,true);active.clear();break;}
          while(next<tasks.size() && active.size()<static_cast<size_t>(workers)) {
            auto name=L"candidate_"+std::to_wstring(next);auto input_file=output/"workers"/(name+L".txt");
            std::ofstream input(input_file);input<<std::setprecision(17);for(double x:tasks[next].start)input<<x<<' ';for(double x:tasks[next].finish)input<<x<<' ';input.close();
            auto result_file=output/"workers"/(name+L".csv");auto args=common;
            args.insert(args.end(),{L"--boundary",input_file.wstring(),L"--worker-output",result_file.wstring()});
            active.push_back(launch(executable,args,result_file,int(next)));++next;
          }
          for(size_t k=0;k<active.size();) {
            if(!finished(active[k])){++k;continue;}auto p=active[k];close_process(p);++completed;active.erase(active.begin()+k);
            if(!fs::exists(p.output))continue;auto connector=read_csv(p.output);const auto& task=tasks[p.task];
            if(!backend.collision_free(connector))continue;
            auto candidate=assemble(original,evasive,task,connector,t0);auto check=validate(backend,candidate,task,connector);
            if(check.join>1e-5||check.dynamics>1e-5||check.bounds>1e-5||!check.collision_free||!check.increasing)continue;
            ++valid_count;double cost=task.start_time-t0+connector.back()[0]+evasive.back()[0]-task.end_time;
            if(cost<best_cost){best_cost=cost;selected=std::move(candidate);validation=check;selected_i=task.i;selected_j=task.j;}
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        status=selected.empty()?"fail_safe_no_connection":"stitched";
      }
      if(selected.empty())selected=brake(original,t0,std::min(t0+1.2,tbrake));
    }
    write_csv(output/"trajectory.csv",selected);
    std::ofstream report(output/"report.json");report<<std::setprecision(17);
    report<<"{\n  \"status\": \""<<status<<"\",\n  \"case_id\": "<<case_id<<",\n  \"completed\": "<<completed<<",\n  \"valid_count\": "<<valid_count
          <<",\n  \"selected_pair\": ["<<selected_i<<", "<<selected_j<<"],\n  \"remaining_time\": "<<selected.back()[0]-t0
          <<",\n  \"join_error\": "<<validation.join<<",\n  \"dynamics_error\": "<<validation.dynamics<<",\n  \"bound_violation\": "<<validation.bounds
          <<",\n  \"collision_free\": "<<(backend.collision_free(selected)?"true":"false")<<",\n  \"wall_seconds\": "<<elapsed()<<"\n}\n";
    std::cout<<status<<" | valid "<<valid_count<<"/"<<completed<<" | pair ("<<selected_i<<","<<selected_j<<") | remaining "<<selected.back()[0]-t0<<" s\n";
    return 0;
  } catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 1;}
}
