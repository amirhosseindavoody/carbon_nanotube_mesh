#ifndef __CNTFUNC_H__
#define __CNTFUNC_H__

#include <math.h>

#define A_CC 2.459512146747806 //lattice constant CNTs

class CNT
{
	int m;
	int n;
	double helicity;
	double diameter;
	double curvature;
	int dr;
	int numAtomsUnitCell;
	double TvecLen;

private:
	void setHelicity();
	void setDiameter();
	void setCurvature();
	void setdr();
	int gcd(int m, int n);
	void setNumAtomsUnitCell();
	void setTvecLen();

public:
	CNT(int hamada_n, int hamada_m);
	int getm() const;
	int getn() const;
	double getDiameter() const;
	double getHelicity() const;
	double getCurvature() const;
	int getNumAtomsUnitCell() const;
	double getTvecLen() const;
};



#endif
