#ifndef PD_FUZZY_H
#define PD_FUZZY_H

#include <Arduino.h>
#include <cmath>

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------VARIABLES-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

extern int MAX_FUZZY_OUTPUT_CONTROL_VOLTAGE;
extern int MAX_FUZZY_INPUT_ERROR;

/*----------------------------------------------------------------------------------------------------------------------------------------------- */
/*--------------------------------------------------------------------FUNCTION-------------------------------------------------------------- */
/*----------------------------------------------------------------------------------------------------------------------------------------------- */

class FuzzyPDController {
public:
  // Constructor: scale dùng để khuếch đại đầu ra, max_error để chuẩn hóa
  FuzzyPDController(double scale = MAX_FUZZY_INPUT_ERROR, double max_err = MAX_FUZZY_INPUT_ERROR) 
    : K_scale(scale), MAX_ERROR(max_err), e_prev(0) {}

  // Gọi hàm này mỗi vòng lặp để cập nhật điều khiển
  double update(double setpoint, double input);

private:
  double e_prev;              // sai số trước đó
  double K_scale;             // tỉ lệ khuếch đại
  double MAX_ERROR;           // sai số tối đa để chuẩn hóa

  double mu_e[5];             // độ membership của e
  double mu_de[5];            // độ membership của de

  // Tập luật: ma trận 5x5 với các giá trị ngôn ngữ đầu ra
  const double rule[5][5] = {
    { -1.0, -1.0, -0.67, -0.33,  0.0   },  // NB
    { -1.0, -0.67, -0.33,  0.0,   0.33  },  // NS
    { -0.67, -0.33,  0.0,   0.33,  0.67  },  // ZE
    { -0.33,  0.0,   0.33,  0.67,  1.0   },  // PS
    {  0.0,   0.33,  0.67,  1.0,   1.0   }   // PB
  };

  // Các hàm thành viên riêng
  void fuzzify(double e_norm, double de_norm);
  double defuzzify();

  // Hàm tam giác và hình thang
  double trimf(double x, double a, double b, double c);
  double trapmf(double x, double a, double b, double c, double d);
};

#endif
