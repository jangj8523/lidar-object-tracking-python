


#include <iostream>
using namespace std;

class PrintObject {
    int width, height;
  public:
    void set_values (int,int);
    int area() {return width*height;}
};

void PrintObject::set_values (int x, int y) {
  width = x;
  height = y;
}
