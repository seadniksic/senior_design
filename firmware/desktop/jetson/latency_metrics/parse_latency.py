f = open("latency_data.txt", 'r')
data_latency =list()
data_compression=list()
for line in f:
    if(line[0:7] == "LATENCY"):
        # print(line[10:-1])
        data_latency.append(line[10:-1])
    elif(line[0:11] == "COMPRESSION"):
        data_compression.append(line[8:-1])
f.close()

out = open('parsed_data.txt', 'w')
out.write("LATENCY\tCOMPRESSION\n")
for i in range(0, len(data)):
    out.write(data_latency[i])
    out.write('\t')
    out.write(data_compression[i])
    out.write('\n')
out.close()
