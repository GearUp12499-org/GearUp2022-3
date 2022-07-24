import re
import json
import csv

with open('telemetry.txt') as f:
    data = [l.strip() for l in f.readlines()]

# print(data)

regex = r'\w+: (\{.*\})$'
name = 'data_'
idx = 0
for line in data:
    m = re.search(regex, line)
    if m:
        # print(m.group(1))
        pairs = m.group(1)[1:-1].split(',')
        idx += 1
        with open(f'{name}{idx}.csv', 'w', newline='') as f:
            w = csv.writer(f)
            for content in pairs:
                parts = [l.strip() for l in content.strip().split(':')]
                w.writerow(parts)
