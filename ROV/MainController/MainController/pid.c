

struct PIDdata {
	float uLast;
	float errLast1, errLast2;
};

float pid(struct PIDdata *plant, float err, float p, float i, float d) {
	float u = plant->uLast + p*(1+i+d) * err - p*(1+2*d) * plant->errLast1 + p*d*plant->errLast2;
	plant->errLast2 = plant->errLast1;
	plant->errLast1 = err;
	plant->uLast = u;
	return u;
}