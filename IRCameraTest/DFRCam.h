#ifndef DFRCAM_H
#define DFRCAM_H
class DFRCamera {
public:
DFRCamera();
~DFRCamera();
void initCamera();
struct Target {
  unsigned int x;
  unsigned int y;
  bool validTarget
};
Target getTarget();
private:
};
#endif
