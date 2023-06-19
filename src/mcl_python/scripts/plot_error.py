
plt.subplot(211)
plt.legend(['100 partículas', '500 partículas', '1000 partículas', '5000 partículas', '10000 partículas', '50000 partículas'], loc=1)
plt.subplot(212)
plt.legend(['100 partículas', '500 partículas', '1000 partículas', '5000 partículas', '10000 partículas', '50000 partículas'], loc=1)
plt.savefig(output_file.replace('.csv', '.png'))
plt.show()