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

Use the sample inputs from `examples/`:

* `examples/fishpool.bmp` (input image)
* `examples/pools.txt` (pool centers & sizes)

> To run, copy these files from `examples/` and place them next to the executable.

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

- [Project report (PDF)](docs/project-report.pdf)  
- [Assignment instructions (PDF, 2022)](docs/escape-project-assignment-2022.PDF)

