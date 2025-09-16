# escape-project

C project implementing image processing and route planning.

## Build
**Linux / macOS**
```bash
gcc group7.c -O2 -Wall -lm -o escape
```

**Windows (MinGW)**

```bash
gcc group7.c -O2 -Wall -o escape.exe
```

## Run

Put the following files next to the executable:

* `fishpool.bmp` (input image)
* `pools.txt` (pool centers & sizes)

Outputs may include:

* `fishpool-copy.bmp`
* `fishpool-fishing.bmp`
* `best-route.txt`

Run:

```bash
./escape    # Linux/macOS
escape.exe  # Windows
```

## Docs

See the project report: [docs/project-report.pdf](docs/project-report.pdf)



