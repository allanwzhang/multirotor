from dryden_python_implementation import Wind_Model

wind_model = Wind_Model(turbulence=5, max_time=600, samples_per_sec=int(60/0.1))

for i in range(50):
    print(wind_model.get_wind_vector(10+i, 50, 7.7, i, [0, 0, 0], True))