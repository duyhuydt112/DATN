#include <PD_Fuzzy.h>

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------VARIABLES-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

int MAX_FUZZY_OUTPUT_CONTROL_VOLTAGE = 12;
int MAX_FUZZY_INPUT_ERROR = 12;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------FUNCTION-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

double FuzzyPDController::trapmf(double x, double a, double b, double c, double d) {
  if (x <= a || x >= d) return 0;
  else if (x >= b && x <= c) return 1;
  else if (x > a && x < b) return (x - a) / (b - a);
  else return (d - x) / (d - c);
}

double FuzzyPDController::trimf(double x, double a, double b, double c) {
  if (x <= a || x >= c) return 0;
  else if (x == b) return 1;
  else if (x < b) return (x - a) / (b - a);
  else return (c - x) / (c - b);
}

void FuzzyPDController::fuzzify(double e_norm, double de_norm) {
  mu_e[0]  = trapmf(e_norm, -1000.0, -1.0, -0.5, -0.1);  // NB
  mu_e[1]  = trimf(e_norm, -0.5, -0.1, 0.0);          // NS
  mu_e[2]  = trimf(e_norm, -0.1, 0.0, 0.1);           // ZE
  mu_e[3]  = trimf(e_norm, 0.0, 0.1, 0.5);            // PS
  mu_e[4]  = trapmf(e_norm, 0.1, 0.5, 1.0, 1000.0);      // PB

  mu_de[0] = trimf(de_norm, -1.5, -1.0, -0.5);        // NB
  mu_de[1] = trimf(de_norm, -1.0, -0.5, 0.0);         // NS
  mu_de[2] = trimf(de_norm, -0.5, 0.0, 0.5);          // ZE
  mu_de[3] = trimf(de_norm, 0.0, 0.5, 1.0);           // PS
  mu_de[4] = trimf(de_norm, 0.5, 1.0, 1.5);           // PB
}

double FuzzyPDController::defuzzify() {
  double num = 0.0, den = 0.0;

  for (int i = 0; i < 5; i++) {
    for (int j = 0; j < 5; j++) {
      double w = mu_e[j] * mu_de[i];  // Độ mạnh của luật
      double rule_out = rule[i][j];   // Đầu ra của luật (giá trị hằng hoặc tuyến tính)

      num += w * rule_out;
      den += w;
    }
  }

  return (den == 0.0) ? 0.0 : num / den;
}

double FuzzyPDController::update(double setpoint, double input) {
  double e_now = setpoint - input;
  double de_now = e_now - e_prev;
  e_prev = e_now;

  double e_norm = constrain(e_now / MAX_ERROR, -1.0, 1.0);
  double de_norm = constrain(de_now / MAX_ERROR, -1.0, 1.0);

  fuzzify(e_norm, de_norm);
  double du = defuzzify();

  return constrain(du * K_scale, -MAX_FUZZY_OUTPUT_CONTROL_VOLTAGE, MAX_FUZZY_OUTPUT_CONTROL_VOLTAGE);
}
