// $GNRMC,,V,,,,,,,,,,M*4E
// $GNRMC,002154.000,V,3632.61109,N,13642.31699,E,0.13,0.00,,,,A*6A
// $GNRMC,002220.000,A,3632.64273,N,13642.30496,E,0.00,0.00,220524,,,A*7B

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

float lng, lat;
char lng_c, lat_c;
int tm;

void parse(char *line)
{
  int p = 0;
  int f = 0;
  char fValid = 0;
  char buf[64];
  int pb = 0;
  while(p < strlen(line)){
    char c = line[p];
    //    printf("%c", c);
    if (c == ','){
      buf[pb] = '\0';
      if (f == 3) lng = atof(buf);
      if (f == 4) lng_c = buf[0];
      if (f == 5) lat = atof(buf);
      if (f == 6) lat_c = buf[0];
      if (f == 9) tm = atoi(buf);
      pb = 0;
      f++;
    }
    else buf[pb++] = c;
    if (f == 2 && c == 'A') fValid = 1;
    p++;
  }
  printf("%f(%c) %f(%c) %d %d\n", lng, lng_c, lat, lat_c, tm, fValid);
}

int main()
{
  char line1[] = "$GNRMC,002220.000,A,3632.64273,N,13642.30496,E,0.00,0.00,220524,,,A*7B";
  char line2[] = "$GNRMC,,V,,,,,,,,,,M*4E";
  char line3[] = "$GNRMC,002154.000,V,3632.61109,N,13642.31699,E,0.13,0.00,,,,A*6A";
  parse(line1);
  parse(line2);
  parse(line3);
		 
}
