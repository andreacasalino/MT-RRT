import os

def getBenchmarkLog():
    path = os.path.join(os.environ['MT_RRT_LOG_PATH'], 'benchmark.json')
    with open(path) as stream:
        return stream.read().strip()
    
if __name__ == '__main__':
    

    path = 
