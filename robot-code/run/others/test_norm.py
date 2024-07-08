import matplotlib.pyplot as plt
import numpy as np

generator = np.random.default_rng()

# Start 0, 1
# End 0, 4
mu, sigma = 0, 1
s = generator.normal(mu, sigma, 1000)

count, bins, ignored = plt.hist(s, 30, density=True)
plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (bins - mu)**2 / (2 * sigma**2) ), linewidth=2, color='r')
plt.show()

# Start 0.06, 0.02
# End 0.06, 2
mu, sigma = 0.06, 2
s = generator.normal(mu, sigma, 1000)

count, bins, ignored = plt.hist(s, 30, density=True)
plt.plot(bins, 1/(sigma * np.sqrt(2 * np.pi)) * np.exp( - (bins - mu)**2 / (2 * sigma**2) ), linewidth=2, color='r')
plt.show()