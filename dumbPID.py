import time

n = 0
last_time = 0

P = [0]
I = [0]
D = [0]
e = [0]
y = [0]
t = [last_time]

try:
    Kp = int(input("Kp:"))
    Ki = int(input("Ki:"))
    Kd = int(input("Kd:"))
except ValueError:
    print("K values must be integers")



for n in range(1, 100000):
    time.sleep(1)
    current_time = time.time()

    e[n] = r[n] - y[n]

    P[n] = Kp * e[n]

    I[n] = I[n-1] + Ki * e[n]

    D[n] = Kd * (e[n] - e[n-1])/(current_time - last_time)

    U = P[n] + I[n] + D[n]