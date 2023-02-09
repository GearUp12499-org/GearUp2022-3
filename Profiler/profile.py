import re
from collections import namedtuple, defaultdict
from ast import literal_eval

from PIL import Image
from tqdm import tqdm

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

    @property
    def roots(self):
        return [x for x, v in self._reverse_graph.items() if len(v) == 0]


HEIGHT = 500
MS_PER_PIXEL = 1
digits = {
    '0': Image.open('digits/0.png'),
    '1': Image.open('digits/1.png'),
    '2': Image.open('digits/2.png'),
    '3': Image.open('digits/3.png'),
    '4': Image.open('digits/4.png'),
    '5': Image.open('digits/5.png'),
    '6': Image.open('digits/6.png'),
    '7': Image.open('digits/7.png'),
    '8': Image.open('digits/8.png'),
    '9': Image.open('digits/9.png'),
}
digit_width = digits['0'].width
digit_height = digits['0'].height

print("loading digits...")
for character, digit in tqdm(digits.items(), total=len(digits)):
    digit.load()
    p = digit.convert('RGBA')
    for x in range(p.width):
        for y in range(p.height):
            if p.getpixel((x, y)) == (0, 0, 0, 255):
                p.putpixel((x, y), (255, 255, 255, 255))
    digits[character] = p


def blit_digits(image: Image.Image, numbers: str, x: int, y: int, color=(0, 0, 0, 255)):
    total_width = len(numbers) * digit_width + 2
    for i in range(x - 1, x + total_width + 1):
        for j in range(y - 1, y + digit_height + 1):
            image.putpixel((i, j), color)

    for i, char in enumerate(numbers):
        image.alpha_composite(digits[char], (x + (i * digit_width), y))


def hsv2rgb(h: float, s: float, v: float):
    """
    HSV to RGB
    :param h: number 0 to 360 degrees
    :param s: 0 to 1
    :param v: 0 to 1
    :return: (r, g, b)
    """
    c = v * s
    x = c * (1 - abs((h * 6) % 2 - 1))
    m = v - c
    r, g, b = 0, 0, 0
    if 0 <= h * 6 < 1:
        r, g, b = c, x, 0
    elif 1 <= h * 6 < 2:
        r, g, b = x, c, 0
    elif 2 <= h * 6 < 3:
        r, g, b = 0, c, x
    elif 3 <= h * 6 < 4:
        r, g, b = 0, x, c
    elif 4 <= h * 6 < 5:
        r, g, b = x, 0, c
    elif 5 <= h * 6 < 6:
        r, g, b = c, 0, x
    r, g, b = (int((r + m) * 255), int((g + m) * 255), int((b + m) * 255))
    return r, g, b


def main():
    with open('profile_match_example.jobprof', 'r') as f:
        d = f.read()
    d = d.split("\n\n")[1:]
    jobs = [parse_job(x) for x in d]
    jobs = {j.identifier: j for j in jobs if j is not None}
    graph = DependencyGraph()
    for j in jobs.values():
        graph.add(j)

    total_millis = max(j.Completed for j in jobs.values()) - min(j.Created for j in jobs.values())
    total_pixels = int(total_millis / MS_PER_PIXEL)
    start = min(j.Created for j in jobs.values())
    print(total_pixels)
    image = Image.new('RGBA', (int(total_pixels) + 1, HEIGHT), color=(0x20, 0x20, 0x20, 0xff))
    BLOCK_COLORS = []
    LABEL_COLORS = []

    S = [0.8, 0.6, 0.4, 0.2]
    # hsv2rgb
    for s in S:
        for i in range(0, 360, int(360 / (len(jobs) / len(S)))):
            BLOCK_COLORS.append(hsv2rgb(i/360, 0.8, s))
            LABEL_COLORS.append(hsv2rgb(i/360, 1, s))

    job_assigned_colors = {}
    label_assigned_colors = {}
    labels = {}
    last_label = 0

    jobs_by_id = list(sorted(jobs.values(), key=lambda x: x.identifier))

    count = 0
    print("drawing...")
    for i, cursor in tqdm(enumerate(range(total_pixels)), total=total_pixels):
        y = 0
        real_cursor = cursor * MS_PER_PIXEL + start
        for job in jobs_by_id:
            if job.Started <= real_cursor < job.Completed:
                percent = job.TotalExecutingTime / job.RunningDuration
                if job.identifier not in job_assigned_colors:
                    job_assigned_colors[job.identifier] = BLOCK_COLORS[count % len(BLOCK_COLORS)]
                    label_assigned_colors[job.identifier] = LABEL_COLORS[count % len(LABEL_COLORS)]
                    count += 1
                if job.identifier not in labels and\
                        (cursor + start) > (job.Started + min(job.TotalExecutingTime / 2, 100)):
                    offs = 2
                    if cursor - last_label < 100:
                        offs += 10
                    labels[job.identifier] = (cursor, y+offs, job.identifier)
                    last_label = cursor
                height = min(int(HEIGHT * percent), HEIGHT - y)
                for k in range(height):
                    image.putpixel((cursor, y + k), job_assigned_colors[job.identifier])
                y += int(height)
    print("drawing labels...")
    for job_id, (x, y, label) in labels.items():
        blit_digits(image, str(label), x, y, color=label_assigned_colors[job_id])
    print("saving...")
    image.save("temp.png")


if __name__ == '__main__':
    main()
