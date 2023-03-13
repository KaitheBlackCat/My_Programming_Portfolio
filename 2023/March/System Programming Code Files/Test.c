#include <stdio.h>

int main(int argc, char *argv[]) {
	double d;
	int conversions;

	printf("Input a floating-point number: ");
	conversions = scanf("%lf", &d);

	printf("There were %d successful conversions\n", conversions);
	printf("You enterd: %f\n", d);

	while(1) {
		
	}

	return 0;
}