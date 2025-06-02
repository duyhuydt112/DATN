#include <Gimbal_High_Planner.h>

// float MPU_GetAngle(){
//     return 10;
// }

/* Sample Time Setup */
double T_Planner = 2.0;
double Ts_Planner = 0.04;
const int MAX_SAMPLES= 100; 

/* Output */
double Planner_OutPut[3][MAX_SAMPLES];
int Number_Samples = 0;

double evaluatePolynomial(const double coeff[6], double t) {
  return coeff[0]*pow(t,5) + coeff[1]*pow(t,4) + coeff[2]*pow(t,3) +
         coeff[3]*pow(t,2) + coeff[4]*t + coeff[5];
}

/* Function Solve 6x6 Matrix */
void solveLinearSystem(double A[6][6], double b[6], double x[6]) {
  double mat[6][7];
  for (int i = 0; i < 6; i++) {
    for (int j = 0; j < 6; j++) mat[i][j] = A[i][j];
    mat[i][6] = b[i];
  }

  for (int i = 0; i < 6; i++) {
    int maxRow = i;
    double maxVal = fabs(mat[i][i]);
    for (int r = i + 1; r < 6; r++) {
      if (fabs(mat[r][i]) > maxVal) {
        maxVal = fabs(mat[r][i]);
        maxRow = r;
      }
    }
    if (maxRow != i) {
      for (int c = i; c < 7; c++) {
        double temp = mat[i][c];
        mat[i][c] = mat[maxRow][c];
        mat[maxRow][c] = temp;
      }
    }

    double div = mat[i][i];
    if (fabs(div) < 1e-12) {
      return;
    }

    for (int j = i; j < 7; j++) mat[i][j] /= div;

    for (int k = 0; k < 6; k++) {
      if (k == i) continue;
      double factor = mat[k][i];
      for (int j = i; j < 7; j++) {
        mat[k][j] -= factor * mat[i][j];
      }
    }
  }

  for (int i = 0; i < 6; i++) {
    x[i] = mat[i][6];
  }
}


void High_Planner(double P[3], double Tilt, double T, double Ts, double q[3][100], int &N_out) {
  double q0[3] = {0, 0, 0}; // pan, tilt, roll

  double q1 = -atan2(P[1], P[0]);
  double q2 = -atan2(P[2], sqrt(P[0]*P[0] + P[1]*P[1]));
  double q3 = -Tilt;
  double qf[3] = {q1, q2, q3};

  int N = int(T / Ts) + 1;
  if (N > MAX_SAMPLES) N = MAX_SAMPLES;  // giới hạn mẫu

  double A[6][6] = {
    {0,     0,     0,     0,    0, 1},
    {0,     0,     0,     0,    1, 0},
    {0,     0,     0,     2,    0, 0},
    {pow(T,5), pow(T,4), pow(T,3), pow(T,2), T, 1},
    {5*pow(T,4), 4*pow(T,3), 3*pow(T,2), 2*T, 1, 0},
    {20*pow(T,3), 12*pow(T,2), 6*T, 2, 0, 0}
  };

  for (int axis = 0; axis < 3; ++axis) {
    double b[6] = { q0[axis], 0, 0, qf[axis], 0, 0 };
    double coeff[6];
    solveLinearSystem(A, b, coeff);

    for (int i = 0; i < N; ++i) {
      double t = i * Ts;
      q[axis][i] = evaluatePolynomial(coeff, t);
    }
  }

  N_out = N;
}

void Planner_Setup(float Duration, float Sample_Time, int Number_of_Samples){
    T_Planner = Duration;
    Ts_Planner = Sample_Time;
    Number_Samples = Number_of_Samples;
}

void Move2_Target_Point(float X, float Y, float Z, float Tilt_Angle){
    double P[3] = {X, Y, Z};
    double Tilt = Tilt_Angle;
    High_Planner(P, Tilt, T_Planner, Ts_Planner, Planner_OutPut, Number_Samples);
}