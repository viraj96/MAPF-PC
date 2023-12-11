#include "TaskAssignment.h"
#include <boost/tokenizer.hpp>

TaskAssignment::TaskAssignment(const string& map_fname,
                               const string& agent_fname, int num_of_agents)
    : Instance() {
  this->map_fname = map_fname;
  this->agent_fname = agent_fname;
  this->num_of_agents = num_of_agents;

  bool succ = false;
  if (map_fname.find("kiva") != string::npos) {
    // We are going to work with KIVA instances
    succ = loadKivaMap();
  } else {
    succ = loadMap();
  }
  if (!succ) {
    cerr << "Map file " << map_fname << " not found." << endl;
    exit(-1);
  }

  if (map_fname.find("kiva") != string::npos) {
    // We are going to load KIVA tasks with implicit precedence constraints
    succ = loadKivaAgentsAndTasks();
  } else {
    succ = loadAgents();
  }
  if (!succ) {
    cerr << "Agent file " << agent_fname << " not found." << endl;
  }
}

bool TaskAssignment::loadKivaMap() {
  using namespace std;
  using namespace boost;

  ifstream file(map_fname.c_str());
  if (!file.is_open()) {
    return false;
  }

  string line;
  tokenizer<char_separator<char>>::iterator begin;

  getline(file, line);
  char_separator<char> sep(",");
  tokenizer<char_separator<char>> tokenizer(line, sep);
  begin = tokenizer.begin();
  num_of_rows = atoi((*begin).c_str()) + 2;  // Read the number of rows
  begin++;
  num_of_cols = atoi((*begin).c_str()) + 2;  // Read the number of columns

  getline(file, line);  // Workpoint number
  getline(file, line);  // Number of agents
  getline(file, line);  // Maximum time

  // Initialize the agent start locations
  int agentNum = 0, endPointNum = 0;
  start_locations.resize(num_of_agents);

  map_size = num_of_cols * num_of_rows;
  my_map.resize(map_size, false);
  for (int i = 1; i < num_of_rows - 1; i++) {
    getline(file, line);
    for (int j = 1; j < num_of_cols - 1; j++) {
      my_map[linearizeCoordinate(i, j)] = (line[j - 1] == '@');
      if (line[j - 1] == 'r') {
        // This is a robot spawn location
        start_locations[agentNum] = linearizeCoordinate(i, j);
        assert(!isObstacle(start_locations[agentNum]));
        agentNum++;
      }
      if (line[j - 1] == 'e') {
        // This is a task spawn location
        end_points.push_back(linearizeCoordinate(i, j));
        assert(!isObstacle(end_points[endPointNum]));
        endPointNum++;
      }
    }
  }

  for (int i = 0; i < num_of_rows; i++) {
    my_map[i * num_of_cols] = false;
    my_map[i * num_of_cols + num_of_cols - 1] = false;
  }
  for (int j = 1; j < num_of_cols - 1; j++) {
    my_map[j] = false;
    my_map[map_size - num_of_cols + j] = false;
  }

  assert(agentNum == num_of_agents);
  file.close();
  return true;
}

bool TaskAssignment::loadKivaAgentsAndTasks() {
  using namespace std;
  using namespace boost;

  ifstream file(agent_fname.c_str());
  if (!file.is_open()) {
    return false;
  }

  string line;
  int taskNum;
  stringstream stringLine;
  getline(file, line);
  stringLine << line;
  stringLine >> taskNum;

  num_of_tasks = 2 * taskNum;
  task_locations.resize(num_of_tasks);

  for (int i = 0; i < num_of_tasks - 1; i += 2) {
    int releaseTime, startTask, goalTask, timeOfStartTask, timeOfGoalTask;
    getline(file, line);
    stringLine.clear();
    stringLine << line;
    stringLine >> releaseTime >> startTask >> goalTask >> timeOfStartTask >>
        timeOfGoalTask;

    startTask %= (int)end_points.size();
    goalTask %= (int)end_points.size();

    task_locations[i] = end_points[startTask];
    task_locations[i + 1] = end_points[goalTask];
    assert(!isObstacle(task_locations[i]));
    assert(!isObstacle(task_locations[i + 1]));

    // temporal_dependecies.emplace_back(i, i + 1);
  }

  cout << "#agents: " << num_of_agents << "\t#tasks: " << num_of_tasks
       << "\t#dependecies: " << (int)temporal_dependecies.size() << endl;

  goal_locations.push_back(task_locations);
  search_engine =
      std::make_unique<MultiLabelSpaceTimeAStar>(*((Instance*)this), 0);
  goal_locations.clear();

  file.close();

  return true;
}

bool TaskAssignment::loadAgents() {

  cout << "Loading " << agent_fname << endl;

  using namespace std;
  using namespace boost;

  string line;
  ifstream myfile(agent_fname.c_str());
  if (!myfile.is_open())
    return false;

  getline(myfile, line);
  // My benchmark
  this->num_of_agents = num_of_agents;
  if (num_of_agents == 0) {
    cerr << "The number of agents should be larger than 0" << endl;
    exit(-1);
  }
  start_locations.resize(num_of_agents);
  // goal_locations.resize(1);
  // temporal_cons.resize(num_of_agents * num_of_agents);
  char_separator<char> sep(",");
  for (int i = 0; i < num_of_agents; i++) {
    getline(myfile, line);
    tokenizer<char_separator<char>> tok(line, sep);
    tokenizer<char_separator<char>>::iterator beg = tok.begin();
    // read start [row,col] for agent i
    auto col = atoi((*beg).c_str());
    beg++;
    auto row = atoi((*beg).c_str());

    start_locations[i] = linearizeCoordinate(row, col);
    assert(!this->isObstacle(start_locations[i]));
  }

  getline(myfile, line);
  while (!myfile.eof() && line[0] != 't') {
    getline(myfile, line);
  }
  getline(myfile, line);

  num_of_tasks = atoi(line.c_str());
  task_locations.resize(num_of_tasks);

  for (int i = 0; i < num_of_tasks; i++) {
    getline(myfile, line);
    tokenizer<char_separator<char>> tok(line, sep);
    tokenizer<char_separator<char>>::iterator beg = tok.begin();
    auto col = atoi((*beg).c_str());
    beg++;
    auto row = atoi((*beg).c_str());

    task_locations[i] = linearizeCoordinate(row, col);
    assert(!this->isObstacle(task_locations[i]));
  }

  while (!myfile.eof() && line[0] != 't') {
    getline(myfile, line);
  }
  getline(myfile, line);

  int num_of_dependencies = atoi(line.c_str());
  task_locations.resize(num_of_tasks);

  for (int i = 0; i < num_of_dependencies; i++) {
    getline(myfile, line);
    tokenizer<char_separator<char>> tok(line, sep);
    tokenizer<char_separator<char>>::iterator beg = tok.begin();
    auto pred = atoi((*beg).c_str());
    beg++;
    auto post = atoi((*beg).c_str());
    temporal_dependecies.push_back({pred, post});
  }

  cout << "#agents: " << num_of_agents << "\t#tasks: " << num_of_tasks
       << "\t#dependecies: " << num_of_dependencies << endl;

  goal_locations.push_back(task_locations);
  search_engine =
      std::make_unique<MultiLabelSpaceTimeAStar>(*((Instance*)this), 0);
  goal_locations.clear();

  myfile.close();

  return true;
}

void TaskAssignment::find_greedy_plan() {

  //
  vector<int> agent_last_timestep(num_of_agents, 0);
  vector<int> agent_last_location = start_locations;
  vector<int> task_complete_timestep(num_of_tasks, -1);
  task_plan.clear();
  task_plan.resize(num_of_agents);

  unordered_map<int, vector<int>> dependent;
  for (auto dependence : temporal_dependecies) {
    int i, j;
    std::tie(i, j) = dependence;
    dependent[j].push_back(i);
  }

  std::priority_queue<pair<int, int>, std::vector<pair<int, int>>,
                      std::greater<pair<int, int>>>
      q;

  for (int i = 0; i < num_of_agents; i++) {
    q.push({0, i});
  }

  int task_cnt = 0;

  while (task_cnt < num_of_tasks/2) {
    int timestep, i;
    std::tie(timestep, i) = q.top();

    cout << "planing for agent " << i << " at timestep " << timestep << endl;

    int loc = agent_last_location[i];
    q.pop();

    int selected_task = -1;
    int selected_task_timestep = -1;
    for (int j = 0; j < num_of_tasks - 1; j+=2) {
      if (task_complete_timestep[j] != -1) {
        continue;
      }
      int task_timestep =
          agent_last_timestep[i] + search_engine->my_heuristic[j][loc];
      bool task_ready = true;

      if (dependent.find(j) != dependent.end()) {
        for (auto k : dependent[j]) {
          if (task_complete_timestep[k] < 0) {
            task_ready = false;
            break;
          }
          task_timestep = max(task_complete_timestep[k], task_timestep);
        }
      }
      if (task_ready &&
          (selected_task == -1 || task_timestep < selected_task_timestep)) {
        selected_task = j;
        selected_task_timestep = task_timestep;
      }
    }
    // assign the task
    if (selected_task != -1) {
      
      cout << "assign tasks " << selected_task << " and " << selected_task + 1 << " to agent " << i << " with distance " << search_engine->my_heuristic[selected_task][loc] + search_engine->my_heuristic[selected_task+1][task_locations[selected_task]] << endl;
      task_plan[i].push_back(selected_task);
      task_plan[i].push_back(selected_task + 1);
      agent_last_timestep[i] = selected_task_timestep + search_engine->my_heuristic[selected_task+1][task_locations[selected_task]];
      task_complete_timestep[selected_task] = selected_task_timestep + search_engine->my_heuristic[selected_task+1][task_locations[selected_task]];
      agent_last_location[i] = task_locations[selected_task + 1];
      
      // cout << "assign task " << selected_task << " to agent " << i
      //      << " with distance "
      //      << search_engine->my_heuristic[selected_task][loc] << endl;
      // task_plan[i].push_back(selected_task);
      // agent_last_timestep[i] = selected_task_timestep;
      // task_complete_timestep[selected_task] = selected_task_timestep;
      // agent_last_location[i] = task_locations[selected_task];
      task_cnt++;
    }
    q.push({agent_last_timestep[i], i});
  }
  
  agent_last_location = start_locations;

  // check whether two agent has same goal location

  bool flag = true;
  while (flag) {
    flag = false;
    for (int i = 0; i < num_of_agents; i++) {
      for (int j = i + 1; j < num_of_agents; j++) {
        if (agent_last_location[i] == agent_last_location[j]) {
          assert(false);
          //   flag = true;
          //   auto task1 = task_plan[i].back();
          //   auto task2 = task_plan[j].back();
          //   if (task_complete_timestep[task1] < task_complete_timestep[task2]){
          //     task_plan[j].pop_back();
          //     task_plan[i].push_back(task2);
          //     agent_last_location[j] = task_locations[task_plan[j].back()];
          //   } else {
          //     task_plan[i].pop_back();
          //     task_plan[j].push_back(task1);
          //     agent_last_location[i] = task_locations[task_plan[i].back()];
          //   }
        }
      }
    }
  }

  // write plan to goal_locations and temporal_dependencies;
  // vector<pair<int, int>> task_to_agent_and_index(num_of_tasks/2, {-1, -1});

  goal_locations.clear();
  goal_locations.resize(num_of_agents);
  for (int i = 0; i < num_of_agents; i++) {
    for (int j = 0; j < task_plan[i].size() - 1; j+=2) {
      auto pickup_idx = task_plan[i][j];
      auto dropoff_idx = task_plan[i][j+1];
      // task_to_agent_and_index[pickup_idx] = {i, j};
      goal_locations[i].push_back(task_locations[pickup_idx]);
      goal_locations[i].push_back(task_locations[dropoff_idx]);
    }
    // if (task_plan[i].size() == 0) {
    goal_locations[i].push_back(start_locations[i]);
    // }
    
  }

  // temporal_cons.clear();
  // temporal_cons.resize(num_of_agents * num_of_agents);
  // for (auto dependence : temporal_dependecies) {
  //   int task_i, task_j, agent_i, i, agent_j, j;
  //   std::tie(task_i, task_j) = dependence;
  //   std::tie(agent_i, i) = task_to_agent_and_index[task_i];
  //   std::tie(agent_j, j) = task_to_agent_and_index[task_j];
  //   assert(agent_i >= 0 && agent_j >= 0);
  //   cout << "temporal dep " << agent_i << "(" << i << ") -> " << agent_j << "("
  //        << j << ")" << endl;
  //   temporal_cons[agent_i * num_of_agents + agent_j].push_back({i, j});
  // }
}
