#include <stdexcept>
#include <string>
#include <vector>

#include "test_framework/generic_test.h"
#include "test_framework/timed_executor.h"
using std::string;
using std::vector;
using std::queue;
using std::pair;

vector<int> g_dx{1, -1, 0, 0};
vector<int> g_dy{0, 0, 1, -1};

bool OutOfBounds(const pair<int, int>& coordinate, int rowSize, int colSize)
{
  return coordinate.first < 0 || coordinate.first >= rowSize || coordinate.second < 0 || coordinate.second >= colSize;
}

void MakeRegionGrey(vector<vector<char>>& board, int x, int y)
{
  // BFS starting at x, y to convert region to grey.
  int rowSize = board.size(), colSize = board[0].size();
  queue<pair<int, int>> bfsQ;
  bfsQ.push({x, y});

  board[x][y] = 'G';

  while (!bfsQ.empty())
  {
    auto current = bfsQ.front();
    bfsQ.pop();

    for (int k = 0; k < g_dx.size(); k++)
    {
      pair<int, int> neighbor{current.first + g_dx[k], current.second + g_dy[k]};
      if (!OutOfBounds(neighbor, rowSize, colSize) &&
        board[neighbor.first][neighbor.second] == 'W')
      {
        bfsQ.push(neighbor);
        board[neighbor.first][neighbor.second] = 'G';
      }
    }
  }
}

void FillSurroundedRegions(vector<vector<char>>* board_ptr) 
{
  auto& board = *board_ptr;
  int rowSize = board.size(), colSize = board[0].size();

  // Convert the white regions starting at borders into grey regions.
  for (int i = 0; i < rowSize; i++)
  {
    for (int j = 0; j < colSize; j = j + colSize - 1)
    {
      if (board[i][j] == 'W')
      {
        MakeRegionGrey(board, i, j);
      }
    }
  }
  for (int j = 0; j < colSize; j++)
  {
    for (int i = 0; i < rowSize; i = i + rowSize - 1)
    {
      if (board[i][j] == 'W')
      {
        MakeRegionGrey(board, i, j);
      }
    }
  }

  // Convert all remaining white cells to black, and grey cells back to white.
  for (int i = 0; i < rowSize; i++)
  {
    for (int j = 0; j < colSize; j++)
    {
      if (board[i][j] == 'W')
      {
        board[i][j] = 'B';
      }
      else if (board[i][j] == 'G')
      {
        board[i][j] = 'W';
      }
    }
  }

  return;
}

vector<vector<string>> FillSurroundedRegionsWrapper(
    TimedExecutor& executor, vector<vector<string>> board) {
  vector<vector<char>> char_vector;
  char_vector.resize(board.size());
  for (int i = 0; i < board.size(); i++) {
    for (const string& s : board[i]) {
      if (s.size() != 1) {
        throw std::runtime_error("String size is not 1");
      }
      char_vector[i].push_back(s[0]);
    }
  }

  executor.Run([&] { FillSurroundedRegions(&char_vector); });

  board.clear();
  board.resize(char_vector.size(), {});
  for (int i = 0; i < board.size(); i++) {
    for (char c : char_vector[i]) {
      board[i].emplace_back(1, c);
    }
  }

  return board;
}

int main(int argc, char* argv[]) {
  std::vector<std::string> args{argv + 1, argv + argc};
  std::vector<std::string> param_names{"executor", "board"};
  return GenericTestMain(
      args, "matrix_enclosed_regions.cc", "matrix_enclosed_regions.tsv",
      &FillSurroundedRegionsWrapper, DefaultComparator{}, param_names);
}
