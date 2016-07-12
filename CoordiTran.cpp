#include "CoordiTran.h"

void eulr2dcm(double eul_vect[3],double DCMbn[3][3])
{
	double phi=-eul_vect[0];
	double theta=-eul_vect[1];
	double psi=-eul_vect[2];
	double cpsi=cos(psi);
	double spsi=sin(psi);
	double cthe=cos(theta);
	double sthe=sin(theta);
	double cphi=cos(phi);
	double sphi=sin(phi);

	double C1[3][3]={cpsi,spsi,0,-spsi,cpsi,0,0,0,1};
	double C2[3][3]={cthe,0,-sthe,0,1,0,sthe,0,cthe};
	double C3[3][3]={1,0,0,0,cphi,sphi,0,-sphi,cphi};
	double temp[3][3];

	double DCMnb[3][3];//´Óµ¼º½×ø±êÏµµ½ÔØÌå×ø±êÏµµÄ·½ÏòÓàÏÒ¾ØÕó
	int i,j,k;

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			temp[i][j]=0;
			for(k=0;k<3;k++)
				temp[i][j] += C2[i][k]*C1[k][j];
		}
	}

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
		{
			DCMnb[i][j]=0;
			for(k=0;k<3;k++)
				DCMnb[i][j] += C3[i][k]*temp[k][j];
		}
	}

	for(i=0;i<3;i++)
	{
		for(j=0;j<3;j++)
			DCMbn[i][j]=DCMnb[j][i];
	}
}

void llh2xyz(double llh[3],double xyz[3])
{
//	double pi=3.141592653589793;
	double phi = llh[0];
	double lambda = llh[1];
	double h = llh[2];

	double a = 6378137.0000;	// earth semimajor axis in meters
	double b = 6356752.3142;	// earth semiminor axis in meters
	double e = sqrt (1-(b/a)*(b/a));

	double sinphi = sin(phi);
	double cosphi = cos(phi);
	double coslam = cos(lambda);
	double sinlam = sin(lambda);
	double tan2phi = (tan(phi))*(tan(phi));
	double tmp = 1 - e*e;
	double tmpden = sqrt( 1 + tmp*tan2phi );

	double x = (a*coslam)/tmpden + h*coslam*cosphi;

	double y = (a*sinlam)/tmpden + h*sinlam*cosphi;

	double tmp2 = sqrt(1 - e*e*sinphi*sinphi);
	double z = (a*tmp*sinphi)/tmp2 + h*sinphi;

	xyz[0] = x;
	xyz[1] = y;
	xyz[2] = z;

}
void xyz2llh(double xyz [3],double llh [3])
{
	double pi=3.141592653589793;
	double x=xyz[0];
	double y=xyz[1];
	double z=xyz[2];
	double x2=x*x;
	double y2=y*y;
	double z2=z*z;

	double a=6378137.0000; //earth radius in meters
	double b=6356752.3142; //earth semiminor in meters
	double e=sqrt(1-(b/a)*(b/a));
	double b2=b*b;
	//cout<<"b2="<<b2<<endl;
	double e2=e*e;
	double ep=e*(a/b);
	//cout<<"ep="<<ep<<endl;
	double r=sqrt(x2+y2);
	//cout<<"r="<<r<<endl;
	double r2 = r*r;
	//cout<<"r2="<<r2<<endl;
	double E2 = a*a - b*b;
	//cout<<"E2="<<E2<<endl;
	double F = 54*b2*z2;
	//cout<<"F="<<F<<endl;
	double G = r2 + (1-e2)*z2 - e2*E2;
	//cout<<"G="<<G<<endl;
	double c = (e2*e2*F*r2)/(G*G*G);
	double s = pow( double (1 + c + sqrt(c*c + 2*c)),double(1.0/3.0));
	//cout<<"s="<<s<<endl;
	double P=F / (3 * (s+1/s+1)*(s+1/s+1) * G*G);
	//cout<<"P="<<P<<endl;
	double Q=sqrt(1+2*e2*e2*P);
	//cout<<"Q="<<Q<<endl;
	double ro=-(P*e2*r)/(1+Q) + sqrt((a*a/2)*(1+1/Q)- (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2);
	//cout<<"ro="<<ro<<endl;
	double tmp=(r-e2*ro)*(r-e2*ro);
	//cout<<"tmp="<<tmp<<endl;
	double U=sqrt(tmp+z2);
	//cout<<"U="<<U<<endl;
	double V=sqrt(tmp+(1-e2)*z2);
	//cout<<"V="<<V<<endl;
	double zo=(b2*z)/(a*V);
	//cout<<"zo="<<zo<<endl;
	double height=U*(a*V-b2)/(a*V);//´Ë´¦¼ÆËã½á¹û»á³ö´í
	//cout<<"U*(1.0-b2/(a*V))="<<U*(1.0-b2/(a*V))<<endl;
	double lat = atan( (z + ep*ep*zo)/r );
	//cout<<"lat="<<lat<<endl;
	double longth;
	double temp = atan(y/x);
	//cout<<"temp="<<temp<<endl;
	if (x >=0)
	{
		longth = temp;
	}
	else if( (x < 0) & (y >= 0) )
	{
		longth = pi + temp;
	}
	else
	{
		longth = temp - pi;
	}

	llh[0] = lat;
	llh[1] = longth;
	llh[2] = height;
}

void xyz2enu(double xyz[3],double orgxyz[3],double enu[3])
{
	double tmpxyz[3];
	double tmporg[3];
	double difxyz[3];
	double orgllh[3];
	double phi,lam,sinphi,cosphi,sinlam,coslam;

	int i;
	for(i=0;i<3;i++)
	{
		tmpxyz[i]=xyz[i];
		tmporg[i]=orgxyz[i];
		difxyz[i]=tmpxyz[i]-tmporg[i];
	}

	xyz2llh(orgxyz,orgllh);

	phi=orgllh[0];
	lam=orgllh[1];
	sinphi=sin(phi);
	cosphi=cos(phi);
	sinlam=sin(lam);
	coslam=cos(lam);
	double R[3][3]={{-sinlam,coslam,0},{-sinphi*coslam,-sinphi*sinlam,cosphi},{cosphi*coslam,cosphi*sinlam,sinphi}};

	enu[0]=0;
	enu[1]=0;
	enu[2]=0;
	for(i=0;i<3;i++)
	{
		enu[0]=enu[0]+R[0][i]*difxyz[i];
		enu[1]=enu[1]+R[1][i]*difxyz[i];
		enu[2]=enu[2]+R[2][i]*difxyz[i];
	}
}

void enu2xyz(double enu[3],double orgxyz[3],double xyz[3])
{
	double orgllh[3];
	xyz2llh(orgxyz,orgllh);

	double phi=orgllh[0];
	double lam=orgllh[1];
	double sinphi=sin(phi);
	double cosphi=cos(phi);
	double sinlam=sin(lam);
	double coslam=cos(lam);

	double R_1[3][3]={-sinlam,-sinphi*coslam,cosphi*coslam,coslam,-sinphi*sinlam,cosphi*sinlam,0,cosphi,sinphi};

	int i;
	int j;

	double difxyz[3];

	for(i=0;i<3;i++)
	{
		difxyz[i]=0;
		for(j=0;j<3;j++)
		{
			difxyz[i]=difxyz[i]+R_1[i][j]*enu[j];
		}

		xyz[i]=orgxyz[i]+difxyz[i];
	}

}
void HDL2enu(double Target_HDL[3],double eulr[3],double Car_Ref[3],double enu[3])
{
	double T[3]={0.058268,1.353135,1.161148};
	double laser_Ri[3][3]={
	   {0.999924333873544,  -0.009067152932191,  -0.008313438834500},
	   {0.009155624811711,   0.999901195675904,   0.010666462403485},
	   {0.008215902984937,  -0.010741770040456,   0.999908552475945}
	};

	double temp1[3];
	double temp2[3];
	double temp3[3];
	double temp_eulr[3];

	int i,j,k;

	for(i=0;i<3;i++)
	{

		temp1[i]=0;
		for(j=0;j<3;j++)
		{
			temp1[i]=temp1[i]+laser_Ri[i][j]*Target_HDL[j];
		}
	}

	temp2[0]=temp1[0]+T[0];
	temp2[1]=temp1[1]+T[1];
	temp2[2]=temp1[1]+T[2];

	double DCMbn[3][3];
	eulr2dcm(temp_eulr,DCMbn);

	for(i=0;i<3;i++)
	{
		enu[i] = Car_Ref[i];
		for(j=0;j<3;j++)
		{
			enu[i] += DCMbn[i][j]*temp2[j];
		}
	}
}


void enu2llh(double enu[3],double orgxyz[3],double llh[3])
{
		double xyz[3]={0,0,0};
		enu2xyz(enu,orgxyz,xyz);
		xyz2llh(xyz,llh);
}

void llh2enu(double llh[3],double orgxyz[3],double enu[3])
{
	double xyz[3]={0,0,0};
	llh2xyz(llh,xyz);
	xyz2enu(xyz,orgxyz,enu);
}

double MappingAngle(double angle)
{
	double pi=3.141592653589793;
	if( angle>=0.0 && angle<=90.0 )
	{
		return (90.0-angle)*pi/180.0;
	}
	else if( angle>90.0 && angle<=270.0 )
	{
		return -(angle-90.0)*pi/180.0;
	}
	else
	{
		return (450.0-angle)*pi/180.0;
	}
}
