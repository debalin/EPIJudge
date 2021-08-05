#include <deque>
#include <vector>

#include "test_framework/generic_test.h"
#include "test_framework/timed_executor.h"
using std::deque;
using std::vector;
using std::queue;
using std::pair;

vector<int> g_dx{1, -1, 0, 0};
vector<int> g_dy{0, 0, 1, -1};

bool OutOfBounds(const pair<int, int>& coordinate, int rowSize, int colSize)
{
  return coordinate.first < 0 || coordinate.first >= rowSize || coordinate.second < 0 || coordinate.second >= colSize;
}

void FlipColor(int x, int y, vector<deque<bool>>* image_ptr) 
{
  auto& image = *image_ptr;

  queue<pair<int, int>> bfsQ;
  int rowSize = image.size(), colSize = image[0].size();
  bool regionColor = image[x][y]; // true is white, false is black.

  // Init queue with the start coordinate.
  bfsQ.push({x, y});
  image[x][y] = !regionColor;
  while (!bfsQ.empty())
  {
    // Pop the front of the queue and set visited.
    auto current = bfsQ.front();
    bfsQ.pop();

    // If the neighbors are within bounds and within region
    // add them to the queue.
    for (size_t i = 0; i < g_dx.size(); i++)
    {
      pair<int, int> neighbor{current.first + g_dx[i], current.second + g_dy[i]};

      if (!OutOfBounds(neighbor, rowSize, colSize) &&
        image[neighbor.first][neighbor.second] == regionColor)
      {
        bfsQ.push(neighbor);
        image[neighbor.first][neighbor.second] = !regionColor;
      }
    }
  }

  return;
}
vector<vector<int>> FlipColorWrapper(TimedExecutor& executor, int x, int y,
                                     vector<vector<int>> image) {
  vector<deque<bool>> b;
  b.reserve(image.size());
  for (const vector<int>& row : image) {
    deque<bool> tmp;
    tmp.resize(row.size());
    for (int i = 0; i < row.size(); ++i) {
      tmp[i] = static_cast<bool>(row[i]);
    }
    b.push_back(tmp);
  }

  executor.Run([&] { FlipColor(x, y, &b); });

  image.resize(b.size());

  for (int i = 0; i < image.size(); ++i) {
    image[i].resize(b.size());
    for (int j = 0; j < image[i].size(); ++j) {
      image[i][j] = b[i][j];
    }
  }
  return image;
}

int main(int argc, char* argv[]) {
  std::vector<std::string> args{argv + 1, argv + argc};
  std::vector<std::string> param_names{"executor", "x", "y", "image"};
  return GenericTestMain(args, "matrix_connected_regions.cc", "painting.tsv",
                         &FlipColorWrapper, DefaultComparator{}, param_names);
}
