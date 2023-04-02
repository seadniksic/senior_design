f = open("latency_compression_data.txt", 'r')
data_latency =list()
data_compression=list()
for line in f:
    if(line[0:7] == "LATENCY"):
        # print(line[10:-1])
        data_latency.append(line[10:-1])
    elif(line[0:11] == "COMPRESSION"):
        data_compression.append(line[12:-1])
f.close()

out = open('parsed_data_2.txt', 'w')
out.write("LATENCY\tCOMPRESSION\n")
for i in range(0, len(data_latency)):
    out.write(data_latency[i])
    out.write('\t')
    out.write(data_compression[i])
    out.write('\n')
out.close()
