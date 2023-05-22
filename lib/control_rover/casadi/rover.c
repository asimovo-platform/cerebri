/* This file was automatically generated by CasADi 3.6.3.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) rover_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_f3 CASADI_PREFIX(f3)
#define casadi_f4 CASADI_PREFIX(f4)
#define casadi_f5 CASADI_PREFIX(f5)
#define casadi_f6 CASADI_PREFIX(f6)
#define casadi_f7 CASADI_PREFIX(f7)
#define casadi_fabs CASADI_PREFIX(fabs)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_sq CASADI_PREFIX(sq)

casadi_real casadi_sq(casadi_real x) { return x*x;}

casadi_real casadi_fabs(casadi_real x) {
/* Pre-c99 compatibility */
#if __STDC_VERSION__ < 199901L
  return x>0 ? x : -x;
#else
  return fabs(x);
#endif
}

static const casadi_int casadi_s0[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[15] = {1, 6, 0, 1, 2, 3, 4, 5, 6, 0, 0, 0, 0, 0, 0};
static const casadi_int casadi_s3[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s4[15] = {3, 3, 0, 3, 6, 9, 0, 1, 2, 0, 1, 2, 0, 1, 2};

/* bezier6_solve:(wp_0[2],wp_1[2],T)->(P[1x6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a1=arg[0]? arg[0][1] : 0;
  a2=5.;
  a3=arg[2]? arg[2][0] : 0;
  a4=(a2/a3);
  a5=(a1/a4);
  a6=-5.;
  a7=(a6/a3);
  a8=(a7/a4);
  a9=(a8*a0);
  a5=(a5-a9);
  if (res[0]!=0) res[0][1]=a5;
  a5=4.;
  a7=(a5*a7);
  a7=(a7/a3);
  a9=(a6/a3);
  a9=(a9-a4);
  a9=(a5*a9);
  a9=(a9/a3);
  a8=(a9*a8);
  a7=(a7+a8);
  a8=(a2/a3);
  a8=(a5*a8);
  a8=(a8/a3);
  a7=(a7/a8);
  a7=(a7*a0);
  a9=(a9/a4);
  a9=(a9/a8);
  a9=(a9*a1);
  a7=(a7-a9);
  if (res[0]!=0) res[0][2]=a7;
  a7=(a6/a3);
  a9=(a2/a3);
  a9=(a7-a9);
  a9=(a5*a9);
  a9=(a9/a3);
  a1=(a9/a7);
  a6=(a6/a3);
  a6=(a5*a6);
  a6=(a6/a3);
  a1=(a1/a6);
  a8=arg[1]? arg[1][1] : 0;
  a1=(a1*a8);
  a2=(a2/a3);
  a4=(a2/a7);
  a9=(a9*a4);
  a5=(a5*a2);
  a5=(a5/a3);
  a9=(a9-a5);
  a9=(a9/a6);
  a6=arg[1]? arg[1][0] : 0;
  a9=(a9*a6);
  a1=(a1-a9);
  if (res[0]!=0) res[0][3]=a1;
  a8=(a8/a7);
  a4=(a4*a6);
  a8=(a8-a4);
  if (res[0]!=0) res[0][4]=a8;
  if (res[0]!=0) res[0][5]=a6;
  return 0;
}

int bezier6_solve(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

int bezier6_solve_alloc_mem(void) {
  return 0;
}

int bezier6_solve_init_mem(int mem) {
  return 0;
}

void bezier6_solve_free_mem(int mem) {
}

int bezier6_solve_checkout(void) {
  return 0;
}

void bezier6_solve_release(int mem) {
}

void bezier6_solve_incref(void) {
}

void bezier6_solve_decref(void) {
}

casadi_int bezier6_solve_n_in(void) { return 3;}

casadi_int bezier6_solve_n_out(void) { return 1;}

casadi_real bezier6_solve_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* bezier6_solve_name_in(casadi_int i) {
  switch (i) {
    case 0: return "wp_0";
    case 1: return "wp_1";
    case 2: return "T";
    default: return 0;
  }
}

const char* bezier6_solve_name_out(casadi_int i) {
  switch (i) {
    case 0: return "P";
    default: return 0;
  }
}

const casadi_int* bezier6_solve_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* bezier6_solve_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

int bezier6_solve_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* bezier6_traj:(t,T,P[1x6])->(r[3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[2]? arg[2][0] : 0;
  a1=1.;
  a2=arg[0]? arg[0][0] : 0;
  a3=arg[1]? arg[1][0] : 0;
  a4=(a2/a3);
  a5=(a1-a4);
  a5=(a0*a5);
  a6=arg[2]? arg[2][1] : 0;
  a7=(a6*a4);
  a5=(a5+a7);
  a7=(a1-a4);
  a5=(a5*a7);
  a7=(a1-a4);
  a7=(a6*a7);
  a8=arg[2]? arg[2][2] : 0;
  a9=(a8*a4);
  a7=(a7+a9);
  a9=(a7*a4);
  a5=(a5+a9);
  a9=(a1-a4);
  a5=(a5*a9);
  a9=(a1-a4);
  a7=(a7*a9);
  a9=(a1-a4);
  a9=(a8*a9);
  a10=arg[2]? arg[2][3] : 0;
  a11=(a10*a4);
  a9=(a9+a11);
  a11=(a9*a4);
  a7=(a7+a11);
  a11=(a7*a4);
  a5=(a5+a11);
  a11=(a1-a4);
  a5=(a5*a11);
  a11=(a1-a4);
  a7=(a7*a11);
  a11=(a1-a4);
  a9=(a9*a11);
  a11=(a1-a4);
  a11=(a10*a11);
  a12=arg[2]? arg[2][4] : 0;
  a13=(a12*a4);
  a11=(a11+a13);
  a13=(a11*a4);
  a9=(a9+a13);
  a13=(a9*a4);
  a7=(a7+a13);
  a13=(a7*a4);
  a5=(a5+a13);
  a13=(a1-a4);
  a5=(a5*a13);
  a13=(a1-a4);
  a7=(a7*a13);
  a13=(a1-a4);
  a9=(a9*a13);
  a13=(a1-a4);
  a11=(a11*a13);
  a13=(a1-a4);
  a13=(a12*a13);
  a14=arg[2]? arg[2][5] : 0;
  a15=(a14*a4);
  a13=(a13+a15);
  a13=(a13*a4);
  a11=(a11+a13);
  a11=(a11*a4);
  a9=(a9+a11);
  a9=(a9*a4);
  a7=(a7+a9);
  a7=(a7*a4);
  a5=(a5+a7);
  if (res[0]!=0) res[0][0]=a5;
  a5=5.;
  a0=(a6-a0);
  a0=(a5*a0);
  a0=(a0/a3);
  a7=(a2/a3);
  a4=(a1-a7);
  a4=(a0*a4);
  a6=(a8-a6);
  a6=(a5*a6);
  a6=(a6/a3);
  a9=(a6*a7);
  a4=(a4+a9);
  a9=(a1-a7);
  a4=(a4*a9);
  a9=(a1-a7);
  a9=(a6*a9);
  a8=(a10-a8);
  a8=(a5*a8);
  a8=(a8/a3);
  a11=(a8*a7);
  a9=(a9+a11);
  a11=(a9*a7);
  a4=(a4+a11);
  a11=(a1-a7);
  a4=(a4*a11);
  a11=(a1-a7);
  a9=(a9*a11);
  a11=(a1-a7);
  a11=(a8*a11);
  a10=(a12-a10);
  a10=(a5*a10);
  a10=(a10/a3);
  a13=(a10*a7);
  a11=(a11+a13);
  a13=(a11*a7);
  a9=(a9+a13);
  a13=(a9*a7);
  a4=(a4+a13);
  a13=(a1-a7);
  a4=(a4*a13);
  a13=(a1-a7);
  a9=(a9*a13);
  a13=(a1-a7);
  a11=(a11*a13);
  a13=(a1-a7);
  a13=(a10*a13);
  a14=(a14-a12);
  a5=(a5*a14);
  a5=(a5/a3);
  a14=(a5*a7);
  a13=(a13+a14);
  a13=(a13*a7);
  a11=(a11+a13);
  a11=(a11*a7);
  a9=(a9+a11);
  a9=(a9*a7);
  a4=(a4+a9);
  if (res[0]!=0) res[0][1]=a4;
  a4=4.;
  a0=(a6-a0);
  a0=(a4*a0);
  a0=(a0/a3);
  a2=(a2/a3);
  a9=(a1-a2);
  a0=(a0*a9);
  a6=(a8-a6);
  a6=(a4*a6);
  a6=(a6/a3);
  a9=(a6*a2);
  a0=(a0+a9);
  a9=(a1-a2);
  a0=(a0*a9);
  a9=(a1-a2);
  a6=(a6*a9);
  a8=(a10-a8);
  a8=(a4*a8);
  a8=(a8/a3);
  a9=(a8*a2);
  a6=(a6+a9);
  a9=(a6*a2);
  a0=(a0+a9);
  a9=(a1-a2);
  a0=(a0*a9);
  a9=(a1-a2);
  a6=(a6*a9);
  a1=(a1-a2);
  a8=(a8*a1);
  a5=(a5-a10);
  a4=(a4*a5);
  a4=(a4/a3);
  a4=(a4*a2);
  a8=(a8+a4);
  a8=(a8*a2);
  a6=(a6+a8);
  a6=(a6*a2);
  a0=(a0+a6);
  if (res[0]!=0) res[0][2]=a0;
  return 0;
}

int bezier6_traj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

int bezier6_traj_alloc_mem(void) {
  return 0;
}

int bezier6_traj_init_mem(int mem) {
  return 0;
}

void bezier6_traj_free_mem(int mem) {
}

int bezier6_traj_checkout(void) {
  return 0;
}

void bezier6_traj_release(int mem) {
}

void bezier6_traj_incref(void) {
}

void bezier6_traj_decref(void) {
}

casadi_int bezier6_traj_n_in(void) { return 3;}

casadi_int bezier6_traj_n_out(void) { return 1;}

casadi_real bezier6_traj_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* bezier6_traj_name_in(casadi_int i) {
  switch (i) {
    case 0: return "t";
    case 1: return "T";
    case 2: return "P";
    default: return 0;
  }
}

const char* bezier6_traj_name_out(casadi_int i) {
  switch (i) {
    case 0: return "r";
    default: return 0;
  }
}

const casadi_int* bezier6_traj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

const casadi_int* bezier6_traj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

int bezier6_traj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* bezier6_rover:(t,T,PX[1x6],PY[1x6])->(x,y,psi,V,omega) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[2]? arg[2][0] : 0;
  a1=1.;
  a2=arg[0]? arg[0][0] : 0;
  a3=arg[1]? arg[1][0] : 0;
  a4=(a2/a3);
  a5=(a1-a4);
  a5=(a0*a5);
  a6=arg[2]? arg[2][1] : 0;
  a7=(a6*a4);
  a5=(a5+a7);
  a7=(a1-a4);
  a5=(a5*a7);
  a7=(a1-a4);
  a7=(a6*a7);
  a8=arg[2]? arg[2][2] : 0;
  a9=(a8*a4);
  a7=(a7+a9);
  a9=(a7*a4);
  a5=(a5+a9);
  a9=(a1-a4);
  a5=(a5*a9);
  a9=(a1-a4);
  a7=(a7*a9);
  a9=(a1-a4);
  a9=(a8*a9);
  a10=arg[2]? arg[2][3] : 0;
  a11=(a10*a4);
  a9=(a9+a11);
  a11=(a9*a4);
  a7=(a7+a11);
  a11=(a7*a4);
  a5=(a5+a11);
  a11=(a1-a4);
  a5=(a5*a11);
  a11=(a1-a4);
  a7=(a7*a11);
  a11=(a1-a4);
  a9=(a9*a11);
  a11=(a1-a4);
  a11=(a10*a11);
  a12=arg[2]? arg[2][4] : 0;
  a13=(a12*a4);
  a11=(a11+a13);
  a13=(a11*a4);
  a9=(a9+a13);
  a13=(a9*a4);
  a7=(a7+a13);
  a13=(a7*a4);
  a5=(a5+a13);
  a13=(a1-a4);
  a5=(a5*a13);
  a13=(a1-a4);
  a7=(a7*a13);
  a13=(a1-a4);
  a9=(a9*a13);
  a13=(a1-a4);
  a11=(a11*a13);
  a13=(a1-a4);
  a13=(a12*a13);
  a14=arg[2]? arg[2][5] : 0;
  a15=(a14*a4);
  a13=(a13+a15);
  a13=(a13*a4);
  a11=(a11+a13);
  a11=(a11*a4);
  a9=(a9+a11);
  a9=(a9*a4);
  a7=(a7+a9);
  a7=(a7*a4);
  a5=(a5+a7);
  if (res[0]!=0) res[0][0]=a5;
  a5=arg[3]? arg[3][0] : 0;
  a7=(a2/a3);
  a4=(a1-a7);
  a4=(a5*a4);
  a9=arg[3]? arg[3][1] : 0;
  a11=(a9*a7);
  a4=(a4+a11);
  a11=(a1-a7);
  a4=(a4*a11);
  a11=(a1-a7);
  a11=(a9*a11);
  a13=arg[3]? arg[3][2] : 0;
  a15=(a13*a7);
  a11=(a11+a15);
  a15=(a11*a7);
  a4=(a4+a15);
  a15=(a1-a7);
  a4=(a4*a15);
  a15=(a1-a7);
  a11=(a11*a15);
  a15=(a1-a7);
  a15=(a13*a15);
  a16=arg[3]? arg[3][3] : 0;
  a17=(a16*a7);
  a15=(a15+a17);
  a17=(a15*a7);
  a11=(a11+a17);
  a17=(a11*a7);
  a4=(a4+a17);
  a17=(a1-a7);
  a4=(a4*a17);
  a17=(a1-a7);
  a11=(a11*a17);
  a17=(a1-a7);
  a15=(a15*a17);
  a17=(a1-a7);
  a17=(a16*a17);
  a18=arg[3]? arg[3][4] : 0;
  a19=(a18*a7);
  a17=(a17+a19);
  a19=(a17*a7);
  a15=(a15+a19);
  a19=(a15*a7);
  a11=(a11+a19);
  a19=(a11*a7);
  a4=(a4+a19);
  a19=(a1-a7);
  a4=(a4*a19);
  a19=(a1-a7);
  a11=(a11*a19);
  a19=(a1-a7);
  a15=(a15*a19);
  a19=(a1-a7);
  a17=(a17*a19);
  a19=(a1-a7);
  a19=(a18*a19);
  a20=arg[3]? arg[3][5] : 0;
  a21=(a20*a7);
  a19=(a19+a21);
  a19=(a19*a7);
  a17=(a17+a19);
  a17=(a17*a7);
  a15=(a15+a17);
  a15=(a15*a7);
  a11=(a11+a15);
  a11=(a11*a7);
  a4=(a4+a11);
  if (res[1]!=0) res[1][0]=a4;
  a4=5.;
  a5=(a9-a5);
  a5=(a4*a5);
  a5=(a5/a3);
  a11=(a2/a3);
  a7=(a1-a11);
  a7=(a5*a7);
  a9=(a13-a9);
  a9=(a4*a9);
  a9=(a9/a3);
  a15=(a9*a11);
  a7=(a7+a15);
  a15=(a1-a11);
  a7=(a7*a15);
  a15=(a1-a11);
  a15=(a9*a15);
  a13=(a16-a13);
  a13=(a4*a13);
  a13=(a13/a3);
  a17=(a13*a11);
  a15=(a15+a17);
  a17=(a15*a11);
  a7=(a7+a17);
  a17=(a1-a11);
  a7=(a7*a17);
  a17=(a1-a11);
  a15=(a15*a17);
  a17=(a1-a11);
  a17=(a13*a17);
  a16=(a18-a16);
  a16=(a4*a16);
  a16=(a16/a3);
  a19=(a16*a11);
  a17=(a17+a19);
  a19=(a17*a11);
  a15=(a15+a19);
  a19=(a15*a11);
  a7=(a7+a19);
  a19=(a1-a11);
  a7=(a7*a19);
  a19=(a1-a11);
  a15=(a15*a19);
  a19=(a1-a11);
  a17=(a17*a19);
  a19=(a1-a11);
  a19=(a16*a19);
  a20=(a20-a18);
  a20=(a4*a20);
  a20=(a20/a3);
  a18=(a20*a11);
  a19=(a19+a18);
  a19=(a19*a11);
  a17=(a17+a19);
  a17=(a17*a11);
  a15=(a15+a17);
  a15=(a15*a11);
  a7=(a7+a15);
  a0=(a6-a0);
  a0=(a4*a0);
  a0=(a0/a3);
  a15=(a2/a3);
  a11=(a1-a15);
  a11=(a0*a11);
  a6=(a8-a6);
  a6=(a4*a6);
  a6=(a6/a3);
  a17=(a6*a15);
  a11=(a11+a17);
  a17=(a1-a15);
  a11=(a11*a17);
  a17=(a1-a15);
  a17=(a6*a17);
  a8=(a10-a8);
  a8=(a4*a8);
  a8=(a8/a3);
  a19=(a8*a15);
  a17=(a17+a19);
  a19=(a17*a15);
  a11=(a11+a19);
  a19=(a1-a15);
  a11=(a11*a19);
  a19=(a1-a15);
  a17=(a17*a19);
  a19=(a1-a15);
  a19=(a8*a19);
  a10=(a12-a10);
  a10=(a4*a10);
  a10=(a10/a3);
  a18=(a10*a15);
  a19=(a19+a18);
  a18=(a19*a15);
  a17=(a17+a18);
  a18=(a17*a15);
  a11=(a11+a18);
  a18=(a1-a15);
  a11=(a11*a18);
  a18=(a1-a15);
  a17=(a17*a18);
  a18=(a1-a15);
  a19=(a19*a18);
  a18=(a1-a15);
  a18=(a10*a18);
  a14=(a14-a12);
  a4=(a4*a14);
  a4=(a4/a3);
  a14=(a4*a15);
  a18=(a18+a14);
  a18=(a18*a15);
  a19=(a19+a18);
  a19=(a19*a15);
  a17=(a17+a19);
  a17=(a17*a15);
  a11=(a11+a17);
  a17=atan2(a7,a11);
  if (res[2]!=0) res[2][0]=a17;
  a17=casadi_sq(a11);
  a15=casadi_sq(a7);
  a17=(a17+a15);
  a15=sqrt(a17);
  if (res[3]!=0) res[3][0]=a15;
  a15=4.;
  a5=(a9-a5);
  a5=(a15*a5);
  a5=(a5/a3);
  a19=(a2/a3);
  a18=(a1-a19);
  a5=(a5*a18);
  a9=(a13-a9);
  a9=(a15*a9);
  a9=(a9/a3);
  a18=(a9*a19);
  a5=(a5+a18);
  a18=(a1-a19);
  a5=(a5*a18);
  a18=(a1-a19);
  a9=(a9*a18);
  a13=(a16-a13);
  a13=(a15*a13);
  a13=(a13/a3);
  a18=(a13*a19);
  a9=(a9+a18);
  a18=(a9*a19);
  a5=(a5+a18);
  a18=(a1-a19);
  a5=(a5*a18);
  a18=(a1-a19);
  a9=(a9*a18);
  a18=(a1-a19);
  a13=(a13*a18);
  a20=(a20-a16);
  a20=(a15*a20);
  a20=(a20/a3);
  a20=(a20*a19);
  a13=(a13+a20);
  a13=(a13*a19);
  a9=(a9+a13);
  a9=(a9*a19);
  a5=(a5+a9);
  a11=(a11*a5);
  a0=(a6-a0);
  a0=(a15*a0);
  a0=(a0/a3);
  a2=(a2/a3);
  a5=(a1-a2);
  a0=(a0*a5);
  a6=(a8-a6);
  a6=(a15*a6);
  a6=(a6/a3);
  a5=(a6*a2);
  a0=(a0+a5);
  a5=(a1-a2);
  a0=(a0*a5);
  a5=(a1-a2);
  a6=(a6*a5);
  a8=(a10-a8);
  a8=(a15*a8);
  a8=(a8/a3);
  a5=(a8*a2);
  a6=(a6+a5);
  a5=(a6*a2);
  a0=(a0+a5);
  a5=(a1-a2);
  a0=(a0*a5);
  a5=(a1-a2);
  a6=(a6*a5);
  a1=(a1-a2);
  a8=(a8*a1);
  a4=(a4-a10);
  a15=(a15*a4);
  a15=(a15/a3);
  a15=(a15*a2);
  a8=(a8+a15);
  a8=(a8*a2);
  a6=(a6+a8);
  a6=(a6*a2);
  a0=(a0+a6);
  a7=(a7*a0);
  a11=(a11-a7);
  a11=(a11/a17);
  if (res[4]!=0) res[4][0]=a11;
  return 0;
}

int bezier6_rover(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

int bezier6_rover_alloc_mem(void) {
  return 0;
}

int bezier6_rover_init_mem(int mem) {
  return 0;
}

void bezier6_rover_free_mem(int mem) {
}

int bezier6_rover_checkout(void) {
  return 0;
}

void bezier6_rover_release(int mem) {
}

void bezier6_rover_incref(void) {
}

void bezier6_rover_decref(void) {
}

casadi_int bezier6_rover_n_in(void) { return 4;}

casadi_int bezier6_rover_n_out(void) { return 5;}

casadi_real bezier6_rover_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* bezier6_rover_name_in(casadi_int i) {
  switch (i) {
    case 0: return "t";
    case 1: return "T";
    case 2: return "PX";
    case 3: return "PY";
    default: return 0;
  }
}

const char* bezier6_rover_name_out(casadi_int i) {
  switch (i) {
    case 0: return "x";
    case 1: return "y";
    case 2: return "psi";
    case 3: return "V";
    case 4: return "omega";
    default: return 0;
  }
}

const casadi_int* bezier6_rover_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

const casadi_int* bezier6_rover_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    case 3: return casadi_s1;
    case 4: return casadi_s1;
    default: return 0;
  }
}

int bezier6_rover_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* ackermann_steering:(L,omega,V)->(delta) */
static int casadi_f3(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1;
  a0=arg[0]? arg[0][0] : 0;
  a1=arg[1]? arg[1][0] : 0;
  a0=(a0*a1);
  a1=arg[2]? arg[2][0] : 0;
  a0=(a0/a1);
  a0=atan(a0);
  if (res[0]!=0) res[0][0]=a0;
  return 0;
}

int ackermann_steering(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f3(arg, res, iw, w, mem);
}

int ackermann_steering_alloc_mem(void) {
  return 0;
}

int ackermann_steering_init_mem(int mem) {
  return 0;
}

void ackermann_steering_free_mem(int mem) {
}

int ackermann_steering_checkout(void) {
  return 0;
}

void ackermann_steering_release(int mem) {
}

void ackermann_steering_incref(void) {
}

void ackermann_steering_decref(void) {
}

casadi_int ackermann_steering_n_in(void) { return 3;}

casadi_int ackermann_steering_n_out(void) { return 1;}

casadi_real ackermann_steering_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* ackermann_steering_name_in(casadi_int i) {
  switch (i) {
    case 0: return "L";
    case 1: return "omega";
    case 2: return "V";
    default: return 0;
  }
}

const char* ackermann_steering_name_out(casadi_int i) {
  switch (i) {
    case 0: return "delta";
    default: return 0;
  }
}

const casadi_int* ackermann_steering_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* ackermann_steering_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

int ackermann_steering_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* differential_steering:(L,omega,w)->(Vw) */
static int casadi_f4(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3;
  a0=arg[0]? arg[0][0] : 0;
  a1=casadi_sq(a0);
  a2=arg[2]? arg[2][0] : 0;
  a3=casadi_sq(a2);
  a1=(a1+a3);
  a1=sqrt(a1);
  a3=2.;
  a1=(a1/a3);
  a3=arg[1]? arg[1][0] : 0;
  a1=(a1*a3);
  a0=(a0/a2);
  a0=atan(a0);
  a0=cos(a0);
  a1=(a1/a0);
  if (res[0]!=0) res[0][0]=a1;
  return 0;
}

int differential_steering(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f4(arg, res, iw, w, mem);
}

int differential_steering_alloc_mem(void) {
  return 0;
}

int differential_steering_init_mem(int mem) {
  return 0;
}

void differential_steering_free_mem(int mem) {
}

int differential_steering_checkout(void) {
  return 0;
}

void differential_steering_release(int mem) {
}

void differential_steering_incref(void) {
}

void differential_steering_decref(void) {
}

casadi_int differential_steering_n_in(void) { return 3;}

casadi_int differential_steering_n_out(void) { return 1;}

casadi_real differential_steering_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* differential_steering_name_in(casadi_int i) {
  switch (i) {
    case 0: return "L";
    case 1: return "omega";
    case 2: return "w";
    default: return 0;
  }
}

const char* differential_steering_name_out(casadi_int i) {
  switch (i) {
    case 0: return "Vw";
    default: return 0;
  }
}

const casadi_int* differential_steering_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    case 1: return casadi_s1;
    case 2: return casadi_s1;
    default: return 0;
  }
}

const casadi_int* differential_steering_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

int differential_steering_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* se2_U:(e[3])->(U[3x3]) */
static int casadi_f5(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0000000000000000e-03;
  a1=arg[0]? arg[0][2] : 0;
  a2=casadi_fabs(a1);
  a2=(a0<a2);
  a3=sin(a1);
  a4=(a1*a3);
  a5=2.;
  a6=cos(a1);
  a7=1.;
  a8=(a6-a7);
  a8=(a5*a8);
  a4=(a4/a8);
  a4=(a2?a4:0);
  a2=(!a2);
  a8=-1.;
  a9=casadi_sq(a1);
  a10=12.;
  a9=(a9/a10);
  a9=(a8+a9);
  a11=casadi_sq(a1);
  a11=casadi_sq(a11);
  a12=720.;
  a11=(a11/a12);
  a9=(a9+a11);
  a2=(a2?a9:0);
  a4=(a4+a2);
  if (res[0]!=0) res[0][0]=a4;
  a2=(a1/a5);
  if (res[0]!=0) res[0][1]=a2;
  a9=0.;
  if (res[0]!=0) res[0][2]=a9;
  a2=(-a2);
  if (res[0]!=0) res[0][3]=a2;
  if (res[0]!=0) res[0][4]=a4;
  if (res[0]!=0) res[0][5]=a9;
  a9=casadi_fabs(a1);
  a9=(a0<a9);
  a4=arg[0]? arg[0][0] : 0;
  a2=(a1*a4);
  a2=(a2*a3);
  a11=(a7-a6);
  a13=arg[0]? arg[0][1] : 0;
  a14=(a1*a13);
  a15=(a5*a4);
  a14=(a14-a15);
  a11=(a11*a14);
  a2=(a2+a11);
  a11=(a5*a1);
  a14=(a7-a6);
  a11=(a11*a14);
  a2=(a2/a11);
  a2=(a9?a2:0);
  a9=(!a9);
  a11=(a13/a5);
  a14=(a1*a4);
  a14=(a14/a10);
  a11=(a11-a14);
  a14=casadi_sq(a1);
  a14=(a1*a14);
  a14=(a14*a4);
  a14=(a14/a12);
  a11=(a11+a14);
  a9=(a9?a11:0);
  a2=(a2+a9);
  if (res[0]!=0) res[0][6]=a2;
  a2=casadi_fabs(a1);
  a0=(a0<a2);
  a2=(a1*a13);
  a2=(a2*a3);
  a3=(a7-a6);
  a9=(a1*a4);
  a11=(a5*a13);
  a9=(a9+a11);
  a3=(a3*a9);
  a2=(a2-a3);
  a3=(a5*a1);
  a7=(a7-a6);
  a3=(a3*a7);
  a2=(a2/a3);
  a2=(a0?a2:0);
  a0=(!a0);
  a4=(a4/a5);
  a5=(a1*a13);
  a5=(a5/a10);
  a4=(a4+a5);
  a5=casadi_sq(a1);
  a1=(a1*a5);
  a1=(a1*a13);
  a1=(a1/a12);
  a4=(a4+a1);
  a4=(-a4);
  a0=(a0?a4:0);
  a2=(a2+a0);
  if (res[0]!=0) res[0][7]=a2;
  if (res[0]!=0) res[0][8]=a8;
  return 0;
}

int se2_U(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f5(arg, res, iw, w, mem);
}

int se2_U_alloc_mem(void) {
  return 0;
}

int se2_U_init_mem(int mem) {
  return 0;
}

void se2_U_free_mem(int mem) {
}

int se2_U_checkout(void) {
  return 0;
}

void se2_U_release(int mem) {
}

void se2_U_incref(void) {
}

void se2_U_decref(void) {
}

casadi_int se2_U_n_in(void) { return 1;}

casadi_int se2_U_n_out(void) { return 1;}

casadi_real se2_U_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* se2_U_name_in(casadi_int i) {
  switch (i) {
    case 0: return "e";
    default: return 0;
  }
}

const char* se2_U_name_out(casadi_int i) {
  switch (i) {
    case 0: return "U";
    default: return 0;
  }
}

const casadi_int* se2_U_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

const casadi_int* se2_U_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

int se2_U_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* se2_U_inv:(e[3])->(U_inv[3x3]) */
static int casadi_f6(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.0000000000000000e-03;
  a1=arg[0]? arg[0][2] : 0;
  a2=casadi_fabs(a1);
  a2=(a0<a2);
  a3=sin(a1);
  a4=(a3/a1);
  a4=(a2?a4:0);
  a2=(!a2);
  a5=1.;
  a6=casadi_sq(a1);
  a7=6.;
  a6=(a6/a7);
  a6=(a5-a6);
  a8=casadi_sq(a1);
  a8=casadi_sq(a8);
  a9=120.;
  a8=(a8/a9);
  a6=(a6+a8);
  a2=(a2?a6:0);
  a4=(a4+a2);
  a2=(-a4);
  if (res[0]!=0) res[0][0]=a2;
  a2=casadi_fabs(a1);
  a2=(a0<a2);
  a6=cos(a1);
  a8=(a5-a6);
  a8=(a8/a1);
  a8=(-a8);
  a8=(a2?a8:0);
  a2=(!a2);
  a10=2.;
  a11=(a1/a10);
  a12=casadi_sq(a1);
  a12=(a1*a12);
  a13=24.;
  a12=(a12/a13);
  a11=(a11-a12);
  a2=(a2?a11:0);
  a8=(a8+a2);
  if (res[0]!=0) res[0][1]=a8;
  a2=0.;
  if (res[0]!=0) res[0][2]=a2;
  a8=(-a8);
  if (res[0]!=0) res[0][3]=a8;
  a4=(-a4);
  if (res[0]!=0) res[0][4]=a4;
  if (res[0]!=0) res[0][5]=a2;
  a2=casadi_fabs(a1);
  a2=(a0<a2);
  a4=arg[0]? arg[0][0] : 0;
  a8=(a1*a6);
  a8=(a8-a1);
  a8=(a8+a3);
  a11=(a10*a3);
  a11=(a11*a6);
  a12=(a11/a10);
  a8=(a8-a12);
  a8=(a4*a8);
  a12=arg[0]? arg[0][1] : 0;
  a14=(a10*a6);
  a15=(a10*a6);
  a15=(a15*a6);
  a15=(a15-a5);
  a16=(a15/a10);
  a14=(a14-a16);
  a16=1.5000000000000000e+00;
  a14=(a14-a16);
  a14=(a12*a14);
  a8=(a8+a14);
  a14=casadi_sq(a1);
  a17=(a5-a6);
  a14=(a14*a17);
  a8=(a8/a14);
  a8=(-a8);
  a8=(a2?a8:0);
  a2=(!a2);
  a14=(a12/a10);
  a17=(a1*a4);
  a17=(a17/a7);
  a14=(a14+a17);
  a17=casadi_sq(a1);
  a17=(a17*a12);
  a17=(a17/a13);
  a14=(a14-a17);
  a17=casadi_sq(a1);
  a17=(a1*a17);
  a17=(a17*a4);
  a17=(a17/a9);
  a14=(a14-a17);
  a17=casadi_sq(a1);
  a17=casadi_sq(a17);
  a17=(a17*a12);
  a18=720.;
  a17=(a17/a18);
  a14=(a14+a17);
  a2=(a2?a14:0);
  a8=(a8+a2);
  a8=(-a8);
  if (res[0]!=0) res[0][6]=a8;
  a8=casadi_fabs(a1);
  a0=(a0<a8);
  a8=-2.;
  a8=(a8*a6);
  a15=(a15/a10);
  a8=(a8+a15);
  a8=(a8+a16);
  a8=(a4*a8);
  a16=(a1*a6);
  a16=(a16-a1);
  a16=(a16+a3);
  a11=(a11/a10);
  a16=(a16-a11);
  a16=(a12*a16);
  a8=(a8+a16);
  a16=casadi_sq(a1);
  a5=(a5-a6);
  a16=(a16*a5);
  a8=(a8/a16);
  a8=(-a8);
  a8=(a0?a8:0);
  a0=(!a0);
  a16=(a1*a12);
  a16=(a16/a7);
  a10=(a4/a10);
  a16=(a16-a10);
  a10=casadi_sq(a1);
  a10=(a10*a4);
  a10=(a10/a13);
  a16=(a16+a10);
  a10=casadi_sq(a1);
  a10=(a1*a10);
  a10=(a10*a12);
  a10=(a10/a9);
  a16=(a16-a10);
  a1=casadi_sq(a1);
  a1=casadi_sq(a1);
  a1=(a1*a4);
  a1=(a1/a18);
  a16=(a16-a1);
  a0=(a0?a16:0);
  a8=(a8+a0);
  a8=(-a8);
  if (res[0]!=0) res[0][7]=a8;
  a8=-1.;
  if (res[0]!=0) res[0][8]=a8;
  return 0;
}

int se2_U_inv(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f6(arg, res, iw, w, mem);
}

int se2_U_inv_alloc_mem(void) {
  return 0;
}

int se2_U_inv_init_mem(int mem) {
  return 0;
}

void se2_U_inv_free_mem(int mem) {
}

int se2_U_inv_checkout(void) {
  return 0;
}

void se2_U_inv_release(int mem) {
}

void se2_U_inv_incref(void) {
}

void se2_U_inv_decref(void) {
}

casadi_int se2_U_inv_n_in(void) { return 1;}

casadi_int se2_U_inv_n_out(void) { return 1;}

casadi_real se2_U_inv_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* se2_U_inv_name_in(casadi_int i) {
  switch (i) {
    case 0: return "e";
    default: return 0;
  }
}

const char* se2_U_inv_name_out(casadi_int i) {
  switch (i) {
    case 0: return "U_inv";
    default: return 0;
  }
}

const casadi_int* se2_U_inv_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

const casadi_int* se2_U_inv_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    default: return 0;
  }
}

int se2_U_inv_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* se2_error:(p[3],r[3])->(error[3]) */
static int casadi_f7(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[1]? arg[1][2] : 0;
  a1=arg[0]? arg[0][2] : 0;
  a0=(a0-a1);
  a2=sin(a0);
  a0=cos(a0);
  a2=atan2(a2,a0);
  a0=casadi_fabs(a2);
  a3=9.9999999999999995e-08;
  a0=(a0<a3);
  a4=1.;
  a5=-1.6666666666666666e-01;
  a6=casadi_sq(a2);
  a5=(a5*a6);
  a5=(a4+a5);
  a6=8.3333333333333332e-03;
  a7=casadi_sq(a2);
  a7=casadi_sq(a7);
  a6=(a6*a7);
  a5=(a5+a6);
  a5=(a0?a5:0);
  a0=(!a0);
  a6=sin(a2);
  a6=(a6/a2);
  a0=(a0?a6:0);
  a5=(a5+a0);
  a0=casadi_sq(a5);
  a6=casadi_fabs(a2);
  a6=(a6<a3);
  a3=5.0000000000000000e-01;
  a3=(a3*a2);
  a7=-4.1666666666666664e-02;
  a8=casadi_sq(a2);
  a8=(a2*a8);
  a7=(a7*a8);
  a3=(a3+a7);
  a7=1.3888888888888889e-03;
  a8=casadi_sq(a2);
  a8=casadi_sq(a8);
  a8=(a2*a8);
  a7=(a7*a8);
  a3=(a3+a7);
  a3=(a6?a3:0);
  a6=(!a6);
  a7=cos(a2);
  a4=(a4-a7);
  a4=(a4/a2);
  a6=(a6?a4:0);
  a3=(a3+a6);
  a6=casadi_sq(a3);
  a0=(a0+a6);
  a6=(a5/a0);
  a4=(-a1);
  a7=cos(a4);
  a8=arg[1]? arg[1][0] : 0;
  a9=(a7*a8);
  a4=sin(a4);
  a10=arg[1]? arg[1][1] : 0;
  a11=(a4*a10);
  a9=(a9-a11);
  a11=cos(a1);
  a12=arg[0]? arg[0][0] : 0;
  a13=(a11*a12);
  a1=sin(a1);
  a14=arg[0]? arg[0][1] : 0;
  a15=(a1*a14);
  a13=(a13+a15);
  a9=(a9-a13);
  a6=(a6*a9);
  a13=(a3/a0);
  a4=(a4*a8);
  a7=(a7*a10);
  a4=(a4+a7);
  a1=(a1*a12);
  a11=(a11*a14);
  a1=(a1-a11);
  a4=(a4+a1);
  a13=(a13*a4);
  a6=(a6+a13);
  if (res[0]!=0) res[0][0]=a6;
  a5=(a5/a0);
  a5=(a5*a4);
  a3=(a3/a0);
  a3=(a3*a9);
  a5=(a5-a3);
  if (res[0]!=0) res[0][1]=a5;
  if (res[0]!=0) res[0][2]=a2;
  return 0;
}

int se2_error(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f7(arg, res, iw, w, mem);
}

int se2_error_alloc_mem(void) {
  return 0;
}

int se2_error_init_mem(int mem) {
  return 0;
}

void se2_error_free_mem(int mem) {
}

int se2_error_checkout(void) {
  return 0;
}

void se2_error_release(int mem) {
}

void se2_error_incref(void) {
}

void se2_error_decref(void) {
}

casadi_int se2_error_n_in(void) { return 2;}

casadi_int se2_error_n_out(void) { return 1;}

casadi_real se2_error_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

const char* se2_error_name_in(casadi_int i) {
  switch (i) {
    case 0: return "p";
    case 1: return "r";
    default: return 0;
  }
}

const char* se2_error_name_out(casadi_int i) {
  switch (i) {
    case 0: return "error";
    default: return 0;
  }
}

const casadi_int* se2_error_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s3;
    default: return 0;
  }
}

const casadi_int* se2_error_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

int se2_error_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
