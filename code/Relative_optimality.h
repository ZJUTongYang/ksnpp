#ifndef _MERGERELATED_
#define _MERGERELATED_

extern int Bro(int Rindex, int Nindex1, int Cindex1, int Nindex2, int Cindex2, int& commonfather);
//extern void pathTakeOverPath(int Rindex);
//extern void pathTakeOverSweeper(int Rindex);
//extern void sweeperTakeOverSweeper(int Rindex);


extern void newPathTakeOverPath(int Rindex, int oldNindex, int oldCindex, int Nindex, int Cindex, double hit_x, double hit_y);
extern void newPathTakeOverSweeper(int Rindex, int gapNindex, int gapCindex, int pathNindex, int pathCindex, double hit_x, double hit_y, int loc);
extern void newSweeperTakeOverSweeper(int Rindex, int oldNindex, int oldCindex, int Nindex, int Cindex, double hit_x, double hit_y);

#endif