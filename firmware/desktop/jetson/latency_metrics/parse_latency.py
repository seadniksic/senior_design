f = open("latency_data.txt", 'r')
data =list()
for line in f:
    if(line[0:7] == "LATENCY"):
        # print(line[10:-1])
        data.append(line[10:-1])
        
f.close()

out = open('parsed_data.txt', 'w')
for i in range(0, len(data)):
    out.write(data[i])
    out.write('\n')
out.close()
