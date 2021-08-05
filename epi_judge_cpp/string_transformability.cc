#include <string>
#include <unordered_set>

#include "test_framework/generic_test.h"
using std::string;
using std::unordered_set;
using std::queue;

struct SequenceCandidate
{
  string S;
  int PathLength;
};

int TransformString(unordered_set<string> D, const string& s, const string& t) 
{
  if (D.find(s) == D.end() ||
    D.find(t) == D.end())
  {
    return -1;
  }  

  queue<SequenceCandidate> bfsQ;
  bfsQ.push({s, 0});
  D.erase(s);

  while (!bfsQ.empty())
  {
    auto& current = bfsQ.front();
    if (current.S == t)
    {
      return current.PathLength;
    }

    for (int i = 0; i < current.S.size(); i++)
    {
      char o = current.S[i];
      for (int c = 0; c < 26; c++)
      {
        current.S[i] = 'a' + c;
        if (D.find(current.S) != D.end())
        {
          bfsQ.push({current.S, current.PathLength + 1});
          D.erase(current.S);
        }
      }
      current.S[i] = o;
    }

    bfsQ.pop();
  }

  return -1;
}

int main(int argc, char* argv[]) {
  std::vector<std::string> args{argv + 1, argv + argc};
  std::vector<std::string> param_names{"D", "s", "t"};
  return GenericTestMain(args, "string_transformability.cc",
                         "string_transformability.tsv", &TransformString,
                         DefaultComparator{}, param_names);
}
