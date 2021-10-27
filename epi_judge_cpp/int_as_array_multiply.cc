#include <vector>

#include "test_framework/generic_test.h"
using std::vector;
vector<int> Multiply(vector<int> num1, vector<int> num2) 
{
  if (num1.empty() || num2.empty())
  {
    return {0};
  }
  bool negative = (num1.front() < 0) ^ (num2.front() < 0);
  num1.front() = std::abs(num1.front());
  num2.front() = std::abs(num2.front());

  vector<int> result(num1.size() + num2.size(), 0);
  for (int i = num1.size() - 1, x = result.size() - 1; i >= 0; i--, x--)
  {
    for (int j = num2.size() - 1, y = x; j >= 0; j--, y--)
    {
      int value = num1[i] * num2[j] + result[y];
      result[y] = value % 10;
      result[y - 1] += value / 10;
    }
  }

  // Trim leading zeros.
  auto nonZeroIt = std::find_if(result.begin(), result.end(), [](const auto& digit) { return digit != 0; });
  result.erase(result.begin(), nonZeroIt);
  if (result.empty())
  {
    return {0};
  }

  // Turn to negative if needed.
  result.front() = negative ? -result.front() : result.front();

  return result;
}

int main(int argc, char* argv[]) {
  std::vector<std::string> args{argv + 1, argv + argc};
  std::vector<std::string> param_names{"num1", "num2"};
  return GenericTestMain(args, "int_as_array_multiply.cc",
                         "int_as_array_multiply.tsv", &Multiply,
                         DefaultComparator{}, param_names);
}
