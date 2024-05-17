function config = Configs()

weights.seepd = 10;
weights.acc = 100;
weights.jerk = 1000;


limits.speed = 35.0;
limits.a = 5.0;
limits.jerk = 10.0;

p_nominal = 0.9;
p_contingency = 1.0 - p_nominal;

time.delta = 0.1;
time.horizon = 8.0;
time.share = 1.0;

init_state.s = 0.0;
init_state.v = 20.0;
init_state.a = 0.0;

cut_in.time = 4.0;
cut_in.speed = init_state.v - 5;
cut_in.s = 50.0;

config.weights = weights;
config.limits = limits;
config.p_nominal = p_nominal;
config.p_contingency = p_contingency;
config.time = time;
config.init_state = init_state;
config.cut_in = cut_in;

end