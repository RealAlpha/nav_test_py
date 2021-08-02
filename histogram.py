import numpy as np
import matplotlib.pyplot as plt
import scipy.stats as stats
import serial

num_samples = 50000

gx_datas = []
with serial.Serial('/dev/tty.usbserial-A900HIOM', 112500) as ser:
    for i in range(num_samples):
        try:
            line = ser.readline().decode().strip('\x00')
        except:
            print("Unable to parse linne! Skipping!")
            continue
        #print(line)
        pairs = line.split(",")
        parse_result = {}
        for pair in pairs:
            parse_result[pair.split(":")[0]] = float(pair.split(":")[1])

        try:
            gx_datas.append(parse_result['gx'])
        except:
            print(f"Invalid parse! Line: {line}")

ser.close()

print("Finished data collection! Dumping & lotting...")
np.array(gx_datas).dump('dump')
plt.hist(gx_datas, bins=50, density=True, stacked=True)  # density=True ensures it's normalized
mean = np.mean(gx_datas)
variance = np.var(gx_datas)
sigma = np.sqrt(variance)  # since var = sigma^2, and we want a + number just take the sqrt
# Draw normal distribution nfor + and - 5sigma
plot_xs = np.linspace(mean - 5*sigma, mean+5*sigma, 1000)
plt.plot(plot_xs, stats.norm.pdf(plot_xs, mean, sigma))
print(f"Mean: {mean}; Variance: {variance}")
plt.show()
print(gx_datas)
