#include <istream>
#include <string>
#include <vector>

#include "test_framework/generic_test.h"
#include "test_framework/serialization_traits.h"
#include "test_framework/test_failure.h"
#include "test_framework/timed_executor.h"
using std::vector;
using std::queue;
using std::unordered_map;
using std::priority_queue;
enum class Color { kWhite, kBlack };
struct Coordinate {
  bool operator==(const Coordinate& that) const {
    return x == that.x && y == that.y;
  }
  bool operator!=(const Coordinate& that) const {
    return x != that.x || y != that.y;
  }
  bool OutOfBounds(size_t rowSize, size_t colSize) const
  {
    return x < 0 || x >= rowSize || y < 0 || y >= colSize;
  }

  int x, y;
};

vector<int> g_dx{1, -1, 0, 0};
vector<int> g_dy{0, 0, 1, -1};

struct hash_coordinate
{
  size_t operator()(const Coordinate& c) const
  {
    return std::hash<int>{}(c.x) ^ std::hash<int>{}(c.y);
  }
};

struct AStarNode
{
  Coordinate C;
  int PathCost = 0;
  int Heuristic = 0;
  int TotalCost = 0;

  AStarNode(const Coordinate& c, int pathCost, int heuristic) :
    C(c), PathCost(pathCost), Heuristic(heuristic)
  {
    TotalCost = PathCost + Heuristic;
  }
};

struct MaxComparator
{
  bool operator()(const AStarNode& n1, const AStarNode& n2)
  {
    return n1.TotalCost > n2.TotalCost;
  }
};

int ManhattanDistance(const Coordinate& c, const Coordinate& e)
{
  return std::abs(c.x - e.x) + std::abs(c.y - e.y);
}

vector<Coordinate> FindWithAStar(
  const vector<vector<Color>>& maze, 
  const Coordinate& start, 
  const Coordinate& end)
{
  priority_queue<AStarNode, vector<AStarNode>, MaxComparator> aStarQ;
  unordered_map<Coordinate, Coordinate, hash_coordinate> predecessor;
  int rowSize = maze.size(), colSize = maze[0].size();
  vector<vector<bool>> visited(rowSize, vector<bool>(colSize, false));
  bool foundPath = false;

  // Init queue. 
  aStarQ.emplace(start, 0, ManhattanDistance(start, end));
  visited[start.x][start.y] = true;
  while (!aStarQ.empty())
  {
    // The current node is showing the best chance of 
    // being the shortest path because the priority queue
    // is maintained as a min heap with the minimum cost 
    // path showing up on the top.
    auto current = aStarQ.top();
    aStarQ.pop();

    if (current.C == end)
    {
      foundPath = true;
      break;
    }

    // If the neighbors are within bounds, not a wall
    // and have not been visited before, add them to the
    // queue.
    for (size_t i = 0; i < g_dx.size(); i++)
    {
      Coordinate neighbor{current.C.x + g_dx[i], current.C.y + g_dy[i]};

      if (!neighbor.OutOfBounds(rowSize, colSize) &&
        maze[neighbor.x][neighbor.y] == Color::kWhite &&
        !visited[neighbor.x][neighbor.y])
      {
        // Store how we reached neighbor from current.
        // Because we won't search through the same node
        // more than once, we can store a 1:1 mapping.
        predecessor[neighbor] = current.C;

        aStarQ.emplace(neighbor, current.PathCost + 1, ManhattanDistance(neighbor, end));
        visited[neighbor.x][neighbor.y] = true;
      }
    }
  }

  if (foundPath)
  {
    // Formulate the result by going through predecessor
    // and collecting all predecessors until start.
    vector<Coordinate> result{{end}};
    Coordinate current = end;
    while (predecessor[current] != start)
    {
      current = predecessor[current];
      result.push_back(current);
    }
    result.push_back(start);
    std::reverse(result.begin(), result.end());

    return result;
  }
  else
  {
    return {};
  }
}

vector<Coordinate> FindWithBFS(
  const vector<vector<Color>>& maze, 
  const Coordinate& start, 
  const Coordinate& end)
{
  queue<Coordinate> bfsQ;
  unordered_map<Coordinate, Coordinate, hash_coordinate> predecessor;
  int rowSize = maze.size(), colSize = maze[0].size();
  vector<vector<bool>> visited(rowSize, vector<bool>(colSize, false));
  bool foundPath = false;

  // Init queue with the start coordinate.
  bfsQ.push(start);
  visited[start.x][start.y] = true;
  while (!bfsQ.empty())
  {
    // Pop the front of the queue and set visited.
    auto current = bfsQ.front();
    bfsQ.pop();

    // If we have reached the end, then exit while loop.
    if (current == end)
    {
      foundPath = true;
      break;
    }

    // If the neighbors are within bounds, not a wall
    // and have not been visited before, add them to the
    // queue.
    for (size_t i = 0; i < g_dx.size(); i++)
    {
      Coordinate neighbor{current.x + g_dx[i], current.y + g_dy[i]};

      if (!neighbor.OutOfBounds(rowSize, colSize) &&
        maze[neighbor.x][neighbor.y] == Color::kWhite &&
        !visited[neighbor.x][neighbor.y])
      {
        // Store how we reached neighbor from current.
        // Because we won't search through the same node
        // more than once, we can store a 1:1 mapping.
        predecessor[neighbor] = current;

        bfsQ.push(neighbor);
        visited[neighbor.x][neighbor.y] = true;
      }
    }
  }

  if (foundPath)
  {
    // Formulate the result by going through predecessor
    // and collecting all predecessors until start.
    vector<Coordinate> result{{end}};
    Coordinate current = end;
    while (predecessor[current] != start)
    {
      current = predecessor[current];
      result.push_back(current);
    }
    result.push_back(start);
    std::reverse(result.begin(), result.end());

    return result;
  }
  else
  {
    return {};
  }
}

vector<Coordinate> FindWithDFS(
  const vector<vector<Color>>& maze, 
  const Coordinate& current, 
  const Coordinate& end,
  vector<vector<bool>>& visited)
{
  if (current == end)
  {
    // Found a path.
    return {end};
  }  

  // Update visited matrix.
  visited[current.x][current.y] = true;

  // Recursively search all neighbors of current coordinate.
  for (size_t i = 0; i < g_dx.size(); i++)
  {
    Coordinate neighbor{current.x + g_dx[i], current.y + g_dy[i]};

    if (!neighbor.OutOfBounds(maze.size(), maze[0].size()) &&
      maze[neighbor.x][neighbor.y] == Color::kWhite &&
      !visited[neighbor.x][neighbor.y])
    {
      auto result = FindWithDFS(maze, neighbor, end, visited);
      if (!result.empty())
      {
        result.push_back(current);
        return result;
      }
    }
  }

  return {};
}

vector<Coordinate> FindWithDFS(
  const vector<vector<Color>>& maze, 
  const Coordinate& start, 
  const Coordinate& end)
{
  vector<vector<bool>> visited(maze.size(), vector<bool>(maze[0].size(), false));
  auto path = FindWithDFS(maze, start, end, visited);
  std::reverse(path.begin(), path.end());

  return path;
}

vector<Coordinate> SearchMaze(vector<vector<Color>> maze, const Coordinate& s,
                              const Coordinate& e) {
  auto path = FindWithDFS(maze, s, e);

  // auto path = FindWithBFS(maze, s, e);

  // auto path = FindWithAStar(maze, s, e);

  return path;
}

namespace test_framework {
template <>
struct SerializationTrait<Color> : SerializationTrait<int> {
  using serialization_type = Color;

  static serialization_type Parse(const json& json_object) {
    return static_cast<serialization_type>(
        SerializationTrait<int>::Parse(json_object));
  }
};
}  // namespace test_framework

namespace test_framework {
template <>
struct SerializationTrait<Coordinate> : UserSerTrait<Coordinate, int, int> {
  static std::vector<std::string> GetMetricNames(const std::string& arg_name) {
    return {};
  }

  static std::vector<int> GetMetrics(const Coordinate& x) { return {}; }
};
}  // namespace test_framework

bool PathElementIsFeasible(const vector<vector<Color>>& maze,
                           const Coordinate& prev, const Coordinate& cur) {
  if (!(0 <= cur.x && cur.x < maze.size() && 0 <= cur.y &&
        cur.y < maze[cur.x].size() && maze[cur.x][cur.y] == Color::kWhite)) {
    return false;
  }
  return cur == Coordinate{prev.x + 1, prev.y} ||
         cur == Coordinate{prev.x - 1, prev.y} ||
         cur == Coordinate{prev.x, prev.y + 1} ||
         cur == Coordinate{prev.x, prev.y - 1};
}

bool SearchMazeWrapper(TimedExecutor& executor,
                       const vector<vector<Color>>& maze, const Coordinate& s,
                       const Coordinate& e) {
  vector<vector<Color>> copy = maze;

  auto path = executor.Run([&] { return SearchMaze(copy, s, e); });

  if (path.empty()) {
    return s == e;
  }

  if (!(path.front() == s) || !(path.back() == e)) {
    throw TestFailure("Path doesn't lay between start and end points");
  }

  for (size_t i = 1; i < path.size(); i++) {
    if (!PathElementIsFeasible(maze, path[i - 1], path[i])) {
      throw TestFailure("Path contains invalid segments");
    }
  }

  return true;
}

int main(int argc, char* argv[]) {
  std::vector<std::string> args{argv + 1, argv + argc};
  std::vector<std::string> param_names{"executor", "maze", "s", "e"};
  return GenericTestMain(args, "search_maze.cc", "search_maze.tsv",
                         &SearchMazeWrapper, DefaultComparator{}, param_names);
}
