#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

int
main()
{
    unsigned segments = 6;
    
    unsigned limit = 7000;
    unsigned dsc_frame=0;
    // creating a wait bar for the calculations! 
    int targetPct = 10;
    unsigned targetCount = limit  / 4 - 1;
    unsigned segPct=0;
    segPct=segments * 10;
    
    for (size_t j = 0; j < limit; j++) {
	if (j == targetCount) {
	    dsc_frame++;
	    cerr << targetPct << "% RPM: " << j << " DSC FRAME" << dsc_frame<<"\n";
	    targetPct += 10;
	    targetCount = limit * targetPct / segPct - 1;
	}
    }
}