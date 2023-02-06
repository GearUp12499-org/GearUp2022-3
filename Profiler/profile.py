import re
from collections import namedtuple, defaultdict
from ast import literal_eval

Job = namedtuple("Job", ("identifier",
                         "tag",
                         "Created",
                         "Started",
                         "Completed",
                         "WaitingDuration",
                         "RunningDuration",
                         "TotalExecutingTime",
                         "TaskTimeSamples",
                         "MeanTaskTime",
                         "P95TaskTime",
                         "MaxTaskTime",
                         "Blocking",
                         "AllTaskTimes"))


def parse_job(details: str):
    if details.strip() == '':
        return
    LABEL_RE = re.compile(r'^ ([A-Za-z0-9]+)=(?:A(\[[\d,. ]*])|D([\d.]+)|([\d.]+))', flags=re.MULTILINE)
    values = {}
    for label_match in LABEL_RE.finditer(details):
        name, *value = label_match.groups()
        value = [x for x in value if x is not None][0]
        value = literal_eval(value)
        values[name] = value
    identifier, tag = re.search(r'ID (\d+) \((.*)\)', details).groups()
    values.update(identifier=int(identifier), tag=tag)
    return Job(**values)


class DependencyGraph:
    def __init__(self):
        self._graph = defaultdict(set)
        self._reverse_graph = defaultdict(set)

    def add(self, job: Job):
        self._graph[job.identifier] = set(job.Blocking)
        self._reverse_graph[job.identifier] = self._reverse_graph[job.identifier] or set()
        for dep in job.Blocking:
            self._reverse_graph[dep].add(job.identifier)


if __name__ == '__main__':
    with open('profile_match_example.jobprof', 'r') as f:
        d = f.read()
    d = d.split("\n\n")[1:]
    jobs = [parse_job(x) for x in d]
    jobs = [j for j in jobs if j is not None]
    graph = DependencyGraph()
    for j in jobs:
        graph.add(j)
        print(j.tag)
    print(graph._reverse_graph)
