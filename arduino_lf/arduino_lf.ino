void citesteSenzori();
void controlMotoare();
void ajustareTraiectorie();


double SA0;
double SA1;
double SA2;
double SA3;
double SA4;
double ajustare;
int output_PD5 = 5;
int output_PD4 = 4;
int input_SA1 = A1;
int output_PD7 = 7;
int input_SA0 = A0;
int output_PD6 = 6;
int input_SA3 = A3;
int input_SA2 = A2;
int input_SA4 = A4;
int led_L = LED_BUILTIN;
int output_PWM10 = 10;
int output_PWM9 = 9;

void citesteSenzori() {
    SA0 = analogRead(input_SA0);
    SA1 = analogRead(input_SA1);
    SA2 = analogRead(input_SA2);
    SA3 = analogRead(input_SA3);
    SA4 = analogRead(input_SA4);
}

void controlMotoare() {
    digitalWrite(output_PD4, (int)1);
    digitalWrite(output_PD5, (int)0);
    digitalWrite(output_PD6, (int)0);
    digitalWrite(output_PD7, (int)1);
    if (ajustare == 0 ) {
        analogWrite(output_PWM9, (int)60);
        analogWrite(output_PWM10, (int)60);
    } else {
        if (ajustare == -1 ) {
            analogWrite(output_PWM9, (int)60);
            analogWrite(output_PWM10, (int)0);
        } else {
            analogWrite(output_PWM9, (int)0);
            analogWrite(output_PWM10, (int)60);
        }
    }
}

void ajustareTraiectorie() {
    if (SA2 < 200 ) {
        ajustare = 0;
    } else {
        if ( (SA0 < 200 ) || (SA1 < 200 ) ) {
            ajustare = 1;
        } else {
            if ( (SA3 < 200 ) || (SA4 < 200 ) ) {
                ajustare = -1;
            }
        }
    }
}

void setup()
{
    pinMode(output_PD5, OUTPUT);
    pinMode(output_PD4, OUTPUT);
    pinMode(input_SA1, INPUT);
    pinMode(output_PD7, OUTPUT);
    pinMode(input_SA0, INPUT);
    pinMode(output_PD6, OUTPUT);
    pinMode(input_SA3, INPUT);
    pinMode(input_SA2, INPUT);
    pinMode(input_SA4, INPUT);
    pinMode(led_L, OUTPUT);
    pinMode(output_PWM10, OUTPUT);
    pinMode(output_PWM9, OUTPUT);
    SA0 = 0;
    SA1 = 0;
    SA2 = 0;
    SA3 = 0;
    SA4 = 0;
    ajustare = 0;
}

void loop()
{
    ajustareTraiectorie();
    citesteSenzori();
    controlMotoare();
    delay(10);
}
