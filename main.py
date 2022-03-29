import argparse
import os
import sys

import uvicorn

parser = argparse.ArgumentParser()
parser.add_argument('--host', type=str, default='0.0.0.0')
parser.add_argument('--port', type=int, default=8000)
parser.add_argument("--workers", type=int, default=1)
parser.add_argument('--reload', action='store_true')


def main():
    args = parser.parse_args()

    if sys.platform == "win32":
        workers = 1
    else:   # ("linux", "darwin")
        workers = min(max(os.cpu_count()+1, args.workers), 32)

    print(r"""
  _____.__                 __    ___.                                      
_/ ____\  |   ____   _____/  |_  \_ |__   ____ _____    ____  ____   ____  
\   __\|  | _/ __ \_/ __ \   __\  | __ \_/ __ \\__  \ _/ ___\/  _ \ /    \ 
 |  |  |  |_\  ___/\  ___/|  |    | \_\ \  ___/ / __ \\  \__(  <_> )   |  \
 |__|  |____/\___  >\___  >__|    |___  /\___  >____  /\___  >____/|___|  /
                 \/     \/            \/     \/     \/     \/           \/ 
2022 Fleet Beacon
    """)
    uvicorn.run('src.fleet_beacon.main:app', host=args.host, port=args.port, reload=args.reload, workers=workers)


if __name__ == "__main__":
    main()
