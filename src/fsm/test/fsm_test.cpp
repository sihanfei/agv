#include <limits.h>
#include <stdio.h>
#include <string.h>

const int range = CHAR_MAX + 1; // 0..CHAR_MAX

struct state
{
  state *transition[range];
  state();
};

struct triple
{
  int from;
  char input;
  int to;
};

class fsm
{
private:
  state *graph;
  state *current;

public:
  void reset();
  void advance(char);
  int end_state();
  int doom_state();
  fsm(triple *);
  virtual ~fsm(); //公有基类的析构函数应该被声明为虚函数
};

state::state()
{
  for (int i      = 0; i < range; ++i)
    transition[i] = NULL;
}

fsm::fsm(triple *p)
{
  int max_node = 0;
  for (triple *e = p; e->from; ++e)
  {
    if (e->from > max_node)
      max_node = e->from;
    if (e->to > max_node)
      max_node = e->to;
  }
  graph = new state[max_node + 1];
  for (triple *e = p; e->from; ++e)
  {
    graph[e->from].transition[e->input] = &graph[e->to];
  }
  current = NULL;
}
fsm::~fsm()
{
  delete[] graph;
}
void fsm::reset()
{
  current = &graph[1];
}

void fsm::advance(char x)
{
  if (current)
    current = current->transition[x];
}

int fsm::end_state()
{
  return current == &graph[0];
}

int fsm::doom_state()
{
  return current == NULL;
}
class sample : public fsm
{
  static triple edges[];

public:
  sample();
};

triple sample::edges[] = {{1, 'A', 2}, {1, 'B', 3}, {1, 'C', 4}, {1, 'D', 5}, {2, 'E', 2}, {2, 'I', 0},
                          {3, 'F', 3}, {3, 'J', 0}, {3, 'M', 4}, {4, 'G', 4}, {4, 'K', 0}, {5, 'H', 5},
                          {5, 'L', 0}, {5, 'O', 2}, {5, 'N', 4}, {0, 0, 0}};
sample::sample() : fsm(edges)
{
}
int main()
{
  char input_string[80];
  printf("Enter input expression:");
  scanf("%s", input_string);
  sample m;
  m.reset();
  int index = 0;
  m.advance(input_string[index++]);
  while (!m.end_state() && !m.doom_state())
  {
    m.advance(input_string[index++]);
  }
  if (m.end_state())
    printf("Valid input expression!/n");
  else
    printf("Invalid input expression!/n");

  return 0;
}