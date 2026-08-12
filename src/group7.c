#include <errno.h>
#include <ctype.h>
#include <float.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define POOL_RED 155
#define POOL_GREEN 190
#define POOL_BLUE 245
#define MIN_POOL_PIXELS 10
#define MOVE_SPEED_MPS 0.2
#define MOVE_FUEL_PER_M 0.2
#define EXTRACTION_FUEL_PER_S 0.2
#define EULER_STEP 0.1
#define COST_ALPHA 2.5
#define DEFAULT_BMP "examples/input/fishpool.bmp"
#define DEFAULT_POOLS "examples/expected/detected-pools.txt"
#define DEFAULT_ROUTE "examples/expected/best-route.txt"
#define DEFAULT_ROUTE_BMP "examples/expected/route-overlay.bmp"
#define DEFAULT_FISH_BMP "examples/expected/fishing-route.bmp"
#define DEFAULT_COST_CSV "examples/expected/cost-profile.csv"

typedef struct {
    unsigned char r;
    unsigned char g;
    unsigned char b;
} Pixel;

typedef struct {
    int x;
    int y;
} Point;

typedef struct {
    int width;
    int height;
    Pixel *pixels;
} Image;

typedef struct {
    Point center;
    int size;
} Pool;

typedef struct {
    int width;
    int height;
    Pool *items;
    size_t count;
} PoolData;

typedef struct {
    Point point;
    int pool_size;
    double time;
    double fuel;
} RouteStop;

typedef struct {
    RouteStop *stops;
    size_t count;
    size_t capacity;
    double total_time;
} Route;

typedef struct {
    const PoolData *data;
    Point destination;
    unsigned char *visited;
    Route current;
    Route best;
    FILE *improvements;
} SearchContext;

static uint16_t read_u16(const unsigned char *p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static uint32_t read_u32(const unsigned char *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int32_t read_i32(const unsigned char *p) {
    return (int32_t)read_u32(p);
}

static void write_u32(unsigned char *p, uint32_t value) {
    p[0] = (unsigned char)(value & 0xffu);
    p[1] = (unsigned char)((value >> 8) & 0xffu);
    p[2] = (unsigned char)((value >> 16) & 0xffu);
    p[3] = (unsigned char)((value >> 24) & 0xffu);
}

static int parse_int(const char *text, int *value) {
    char *end = NULL;
    long parsed;
    errno = 0;
    parsed = strtol(text, &end, 10);
    while (end && isspace((unsigned char)*end)) ++end;
    if (errno != 0 || end == text || *end != '\0' || parsed < INT32_MIN || parsed > INT32_MAX) {
        return 0;
    }
    *value = (int)parsed;
    return 1;
}

static int parse_double(const char *text, double *value) {
    char *end = NULL;
    errno = 0;
    *value = strtod(text, &end);
    while (end && isspace((unsigned char)*end)) ++end;
    return errno == 0 && end != text && *end == '\0' &&
           *value <= DBL_MAX && *value >= -DBL_MAX;
}

static int parse_point(const char *text, Point *point) {
    int consumed = 0;
    if (sscanf(text, " %d , %d %n", &point->x, &point->y, &consumed) != 2) return 0;
    while (isspace((unsigned char)text[consumed])) ++consumed;
    return text[consumed] == '\0';
}

static double point_distance(Point a, Point b) {
    return hypot((double)b.x - a.x, (double)b.y - a.y);
}

static size_t image_index(const Image *image, int x, int y) {
    return (size_t)(y - 1) * (size_t)image->width + (size_t)(x - 1);
}

static Pixel *image_pixel(Image *image, int x, int y) {
    if (!image || x < 1 || x > image->width || y < 1 || y > image->height) {
        return NULL;
    }
    return &image->pixels[image_index(image, x, y)];
}

static const Pixel *image_pixel_const(const Image *image, int x, int y) {
    if (!image || x < 1 || x > image->width || y < 1 || y > image->height) {
        return NULL;
    }
    return &image->pixels[image_index(image, x, y)];
}

static void free_image(Image *image) {
    if (image) {
        free(image->pixels);
        image->pixels = NULL;
        image->width = 0;
        image->height = 0;
    }
}

static int load_bmp(const char *path, Image *image) {
    unsigned char header[54];
    FILE *file = fopen(path, "rb");
    uint32_t offset;
    int32_t raw_width;
    int32_t raw_height;
    int top_down;
    size_t row_size;
    unsigned char *row = NULL;

    memset(image, 0, sizeof(*image));
    if (!file) {
        fprintf(stderr, "Error: cannot open BMP file '%s'.\n", path);
        return 0;
    }
    if (fread(header, 1, sizeof(header), file) != sizeof(header) ||
        header[0] != 'B' || header[1] != 'M' || read_u32(header + 14) < 40 ||
        read_u16(header + 26) != 1 || read_u16(header + 28) != 24 ||
        read_u32(header + 30) != 0) {
        fprintf(stderr, "Error: '%s' is not an uncompressed 24-bit BMP.\n", path);
        fclose(file);
        return 0;
    }

    offset = read_u32(header + 10);
    raw_width = read_i32(header + 18);
    raw_height = read_i32(header + 22);
    if (raw_width <= 0 || raw_height == 0 || raw_height == INT32_MIN) {
        fprintf(stderr, "Error: unsupported BMP dimensions in '%s'.\n", path);
        fclose(file);
        return 0;
    }

    image->width = raw_width;
    image->height = raw_height < 0 ? -raw_height : raw_height;
    top_down = raw_height < 0;
    if ((size_t)image->width > SIZE_MAX / (size_t)image->height ||
        (size_t)image->width * (size_t)image->height > SIZE_MAX / sizeof(Pixel)) {
        fprintf(stderr, "Error: BMP dimensions are too large.\n");
        fclose(file);
        return 0;
    }
    image->pixels = calloc((size_t)image->width * (size_t)image->height, sizeof(Pixel));
    row_size = ((size_t)image->width * 3u + 3u) & ~(size_t)3u;
    row = malloc(row_size);
    if (!image->pixels || !row || fseek(file, (long)offset, SEEK_SET) != 0) {
        fprintf(stderr, "Error: unable to allocate or seek while reading '%s'.\n", path);
        free(row);
        free_image(image);
        fclose(file);
        return 0;
    }

    for (int file_row = 0; file_row < image->height; ++file_row) {
        int y = top_down ? image->height - file_row : file_row + 1;
        if (fread(row, 1, row_size, file) != row_size) {
            fprintf(stderr, "Error: truncated BMP pixel data in '%s'.\n", path);
            free(row);
            free_image(image);
            fclose(file);
            return 0;
        }
        for (int x = 1; x <= image->width; ++x) {
            Pixel *pixel = image_pixel(image, x, y);
            size_t base = (size_t)(x - 1) * 3u;
            pixel->b = row[base];
            pixel->g = row[base + 1];
            pixel->r = row[base + 2];
        }
    }

    free(row);
    fclose(file);
    return 1;
}

static int save_bmp(const char *path, const Image *image) {
    unsigned char header[54] = {0};
    size_t row_size = ((size_t)image->width * 3u + 3u) & ~(size_t)3u;
    size_t image_size = row_size * (size_t)image->height;
    unsigned char *row = calloc(row_size, 1);
    FILE *file;

    if (!row || image_size > UINT32_MAX - 54u) {
        free(row);
        return 0;
    }
    file = fopen(path, "wb");
    if (!file) {
        fprintf(stderr, "Error: cannot create BMP file '%s'.\n", path);
        free(row);
        return 0;
    }

    header[0] = 'B';
    header[1] = 'M';
    write_u32(header + 2, (uint32_t)(54u + image_size));
    write_u32(header + 10, 54u);
    write_u32(header + 14, 40u);
    write_u32(header + 18, (uint32_t)image->width);
    write_u32(header + 22, (uint32_t)image->height);
    header[26] = 1;
    header[28] = 24;
    write_u32(header + 34, (uint32_t)image_size);
    write_u32(header + 38, 2835u);
    write_u32(header + 42, 2835u);

    if (fwrite(header, 1, sizeof(header), file) != sizeof(header)) {
        fclose(file);
        free(row);
        return 0;
    }
    for (int y = 1; y <= image->height; ++y) {
        memset(row, 0, row_size);
        for (int x = 1; x <= image->width; ++x) {
            const Pixel *pixel = image_pixel_const(image, x, y);
            size_t base = (size_t)(x - 1) * 3u;
            row[base] = pixel->b;
            row[base + 1] = pixel->g;
            row[base + 2] = pixel->r;
        }
        if (fwrite(row, 1, row_size, file) != row_size) {
            fclose(file);
            free(row);
            return 0;
        }
    }
    fclose(file);
    free(row);
    return 1;
}

static int is_pool_pixel(const Pixel *pixel) {
    return pixel && pixel->r == POOL_RED && pixel->g == POOL_GREEN && pixel->b == POOL_BLUE;
}

static int append_pool(PoolData *data, Pool pool) {
    Pool *grown = realloc(data->items, (data->count + 1u) * sizeof(*data->items));
    if (!grown) {
        return 0;
    }
    data->items = grown;
    data->items[data->count++] = pool;
    return 1;
}

static int detect_pools(const Image *image, PoolData *data) {
    size_t total = (size_t)image->width * (size_t)image->height;
    unsigned char *visited = calloc(total, 1);
    int *queue = malloc(total * sizeof(*queue));
    memset(data, 0, sizeof(*data));
    data->width = image->width;
    data->height = image->height;
    if (!visited || !queue) {
        free(visited);
        free(queue);
        return 0;
    }

    for (int y = 1; y <= image->height; ++y) {
        for (int x = 1; x <= image->width; ++x) {
            size_t start = image_index(image, x, y);
            size_t head = 0;
            size_t tail = 0;
            int count = 0;
            int min_x = x, max_x = x, min_y = y, max_y = y;
            if (visited[start] || !is_pool_pixel(image_pixel_const(image, x, y))) {
                continue;
            }
            visited[start] = 1;
            queue[tail++] = (int)start;
            while (head < tail) {
                int index = queue[head++];
                int px = index % image->width + 1;
                int py = index / image->width + 1;
                const int dx[4] = {1, -1, 0, 0};
                const int dy[4] = {0, 0, 1, -1};
                ++count;
                if (px < min_x) min_x = px;
                if (px > max_x) max_x = px;
                if (py < min_y) min_y = py;
                if (py > max_y) max_y = py;
                for (int direction = 0; direction < 4; ++direction) {
                    int nx = px + dx[direction];
                    int ny = py + dy[direction];
                    if (nx >= 1 && nx <= image->width && ny >= 1 && ny <= image->height) {
                        size_t neighbor = image_index(image, nx, ny);
                        if (!visited[neighbor] && is_pool_pixel(image_pixel_const(image, nx, ny))) {
                            visited[neighbor] = 1;
                            queue[tail++] = (int)neighbor;
                        }
                    }
                }
            }
            if (count >= MIN_POOL_PIXELS) {
                Pool pool = {{(min_x + max_x) / 2, (min_y + max_y) / 2}, count};
                if (!append_pool(data, pool)) {
                    free(visited);
                    free(queue);
                    free(data->items);
                    memset(data, 0, sizeof(*data));
                    return 0;
                }
            }
        }
    }
    free(visited);
    free(queue);
    return 1;
}

static void free_pool_data(PoolData *data) {
    free(data->items);
    memset(data, 0, sizeof(*data));
}

static int write_pools(const char *path, const PoolData *data) {
    FILE *file = fopen(path, "w");
    if (!file) {
        fprintf(stderr, "Error: cannot create pool data file '%s'.\n", path);
        return 0;
    }
    fprintf(file, "Image size (%dx%d)\n", data->width, data->height);
    fprintf(file, "Pool Center\tSize\n");
    fprintf(file, "===========\t====\n");
    for (size_t i = 0; i < data->count; ++i) {
        fprintf(file, "(%d,%d)\t%d\n", data->items[i].center.x, data->items[i].center.y,
                data->items[i].size);
    }
    fclose(file);
    return 1;
}

static int read_pools(const char *path, PoolData *data) {
    FILE *file = fopen(path, "r");
    char line[256];
    memset(data, 0, sizeof(*data));
    if (!file) {
        fprintf(stderr, "Error: cannot open pool data file '%s'.\n", path);
        return 0;
    }
    if (!fgets(line, sizeof(line), file) ||
        sscanf(line, "Image size (%dx%d)", &data->width, &data->height) != 2 ||
        data->width <= 0 || data->height <= 0 || !fgets(line, sizeof(line), file) ||
        !fgets(line, sizeof(line), file)) {
        fprintf(stderr, "Error: invalid header in '%s'.\n", path);
        fclose(file);
        return 0;
    }
    while (fgets(line, sizeof(line), file)) {
        Pool pool;
        char extra;
        if (line[0] == '\n' || line[0] == '\r') {
            continue;
        }
        if (sscanf(line, " (%d , %d) %d %c", &pool.center.x, &pool.center.y, &pool.size, &extra) != 3 ||
            pool.center.x < 1 || pool.center.x > data->width ||
            pool.center.y < 1 || pool.center.y > data->height || pool.size < MIN_POOL_PIXELS ||
            !append_pool(data, pool)) {
            fprintf(stderr, "Error: invalid pool record in '%s': %s", path, line);
            fclose(file);
            free_pool_data(data);
            return 0;
        }
    }
    fclose(file);
    return 1;
}

static int compare_pool_size_desc(const void *left, const void *right) {
    const Pool *a = left;
    const Pool *b = right;
    if (a->size != b->size) return b->size - a->size;
    if (a->center.y != b->center.y) return a->center.y - b->center.y;
    return a->center.x - b->center.x;
}

static void print_sorted_pools(PoolData *data) {
    qsort(data->items, data->count, sizeof(*data->items), compare_pool_size_desc);
    printf("Sorted pools by size:\n");
    printf("Coordinate   Size\n");
    printf("==========   ====\n");
    for (size_t i = 0; i < data->count; ++i) {
        printf("(%3d,%3d)   %d\n", data->items[i].center.x, data->items[i].center.y,
               data->items[i].size);
    }
}

static int route_reserve(Route *route, size_t needed) {
    RouteStop *grown;
    size_t capacity = route->capacity ? route->capacity : 8;
    while (capacity < needed) capacity *= 2;
    if (capacity == route->capacity) return 1;
    grown = realloc(route->stops, capacity * sizeof(*route->stops));
    if (!grown) return 0;
    route->stops = grown;
    route->capacity = capacity;
    return 1;
}

static int route_push(Route *route, RouteStop stop) {
    if (!route_reserve(route, route->count + 1u)) return 0;
    route->stops[route->count++] = stop;
    route->total_time = stop.time;
    return 1;
}

static int route_copy(Route *destination, const Route *source) {
    if (!route_reserve(destination, source->count)) return 0;
    memcpy(destination->stops, source->stops, source->count * sizeof(*source->stops));
    destination->count = source->count;
    destination->total_time = source->total_time;
    return 1;
}

static void print_route_line(FILE *file, const Route *route) {
    for (size_t i = 0; i < route->count; ++i) {
        const RouteStop *stop = &route->stops[i];
        fprintf(file, "Time=%.2f (%d,%d) fuel=%.2f%s", stop->time, stop->point.x,
                stop->point.y, stop->fuel, i + 1 == route->count ? "\n" : " -> ");
    }
}

static int nearest_unvisited(const SearchContext *context, Point current, int output[2]) {
    double distances[2] = {HUGE_VAL, HUGE_VAL};
    output[0] = output[1] = -1;
    for (size_t i = 0; i < context->data->count; ++i) {
        double distance;
        if (context->visited[i]) continue;
        distance = point_distance(current, context->data->items[i].center);
        if (distance < distances[0] || (distance == distances[0] && (int)i < output[0])) {
            distances[1] = distances[0];
            output[1] = output[0];
            distances[0] = distance;
            output[0] = (int)i;
        } else if (distance < distances[1] || (distance == distances[1] && (int)i < output[1])) {
            distances[1] = distance;
            output[1] = (int)i;
        }
    }
    return output[0] >= 0 ? (output[1] >= 0 ? 2 : 1) : 0;
}

static int search_routes(SearchContext *context) {
    RouteStop current = context->current.stops[context->current.count - 1];
    double direct_distance = point_distance(current.point, context->destination);
    int neighbors[2];
    int neighbor_count;

    if (current.fuel + 1e-9 >= MOVE_FUEL_PER_M * direct_distance) {
        RouteStop destination = {context->destination, 0,
            current.time + direct_distance / MOVE_SPEED_MPS,
            current.fuel - MOVE_FUEL_PER_M * direct_distance};
        if (!route_push(&context->current, destination)) return 0;
        if (context->best.count == 0 || context->current.total_time < context->best.total_time - 1e-9) {
            if (!route_copy(&context->best, &context->current)) return 0;
            print_route_line(stdout, &context->current);
            if (context->improvements) print_route_line(context->improvements, &context->current);
        }
        --context->current.count;
    }

    neighbor_count = nearest_unvisited(context, current.point, neighbors);
    for (int n = 0; n < neighbor_count; ++n) {
        int index = neighbors[n];
        const Pool *pool = &context->data->items[index];
        double distance = point_distance(current.point, pool->center);
        RouteStop stop;
        if (current.fuel + 1e-9 < MOVE_FUEL_PER_M * distance) continue;
        stop.point = pool->center;
        stop.pool_size = pool->size;
        stop.time = current.time + distance / MOVE_SPEED_MPS + pool->size;
        stop.fuel = current.fuel - MOVE_FUEL_PER_M * distance + EXTRACTION_FUEL_PER_S * pool->size;
        context->visited[index] = 1;
        if (!route_push(&context->current, stop) || !search_routes(context)) {
            context->visited[index] = 0;
            return 0;
        }
        --context->current.count;
        context->visited[index] = 0;
    }
    return 1;
}

static int write_route(const char *path, const Route *route, int width, int height) {
    FILE *file = fopen(path, "w");
    if (!file) {
        fprintf(stderr, "Error: cannot create route file '%s'.\n", path);
        return 0;
    }
    fprintf(file, "Map size (%dx%d)\n", width, height);
    fprintf(file, "Best Route\tPool Size\tTime\tFuel\n");
    fprintf(file, "==========\t=========\t====\t====\n");
    for (size_t i = 0; i < route->count; ++i) {
        const RouteStop *stop = &route->stops[i];
        fprintf(file, "(%d,%d)\t%d\t%.6f\t%.6f\n", stop->point.x, stop->point.y,
                stop->pool_size, stop->time, stop->fuel);
    }
    fclose(file);
    return 1;
}

static int read_route(const char *path, Route *route, int *width, int *height) {
    FILE *file = fopen(path, "r");
    char line[256];
    memset(route, 0, sizeof(*route));
    if (!file || !fgets(line, sizeof(line), file) ||
        sscanf(line, "Map size (%dx%d)", width, height) != 2 ||
        !fgets(line, sizeof(line), file) || !fgets(line, sizeof(line), file)) {
        fprintf(stderr, "Error: invalid or missing route file '%s'.\n", path);
        if (file) fclose(file);
        return 0;
    }
    while (fgets(line, sizeof(line), file)) {
        RouteStop stop;
        char extra;
        if (sscanf(line, " (%d , %d) %d %lf %lf %c", &stop.point.x, &stop.point.y,
                   &stop.pool_size, &stop.time, &stop.fuel, &extra) != 5 || !route_push(route, stop)) {
            fprintf(stderr, "Error: invalid route record in '%s'.\n", path);
            fclose(file);
            free(route->stops);
            memset(route, 0, sizeof(*route));
            return 0;
        }
    }
    fclose(file);
    return route->count >= 2;
}

static void draw_line(Image *image, Point start, Point end, Pixel color) {
    int x = start.x, y = start.y;
    int dx = abs(end.x - start.x), sx = start.x < end.x ? 1 : -1;
    int dy = -abs(end.y - start.y), sy = start.y < end.y ? 1 : -1;
    int error = dx + dy;
    for (;;) {
        Pixel *pixel = image_pixel(image, x, y);
        if (pixel) *pixel = color;
        if (x == end.x && y == end.y) break;
        if (2 * error >= dy) { error += dy; x += sx; }
        if (2 * error <= dx) { error += dx; y += sy; }
    }
}

static int draw_route_overlay(const char *input_bmp, const char *output_bmp, const Route *route,
                              Pixel line_color, Pixel stop_color) {
    Image image;
    Pixel start_color = {220, 38, 38};
    Pixel destination_color = {22, 163, 74};
    if (!load_bmp(input_bmp, &image)) return 0;
    for (size_t i = 1; i < route->count; ++i) {
        draw_line(&image, route->stops[i - 1].point, route->stops[i].point, line_color);
    }
    for (size_t i = 1; i + 1 < route->count; ++i) {
        Pixel *pixel = image_pixel(&image, route->stops[i].point.x, route->stops[i].point.y);
        if (pixel) *pixel = stop_color;
    }
    Pixel *start = image_pixel(&image, route->stops[0].point.x, route->stops[0].point.y);
    Pixel *destination = image_pixel(&image, route->stops[route->count - 1].point.x,
                                     route->stops[route->count - 1].point.y);
    if (start) *start = start_color;
    if (destination) *destination = destination_color;
    int ok = save_bmp(output_bmp, &image);
    free_image(&image);
    return ok;
}

static int run_scan(const char *bmp_path, const char *pool_path) {
    Image image;
    PoolData data;
    if (!load_bmp(bmp_path, &image)) return 0;
    if (!detect_pools(&image, &data)) {
        fprintf(stderr, "Error: pool detection failed.\n");
        free_image(&image);
        return 0;
    }
    printf("Image size: %dx%d\n", data.width, data.height);
    printf("Detected pools: %llu\n", (unsigned long long)data.count);
    for (size_t i = 0; i < data.count; ++i) {
        printf("  Pool %llu: center=(%d,%d), size=%d\n", (unsigned long long)(i + 1u),
               data.items[i].center.x, data.items[i].center.y, data.items[i].size);
    }
    int ok = write_pools(pool_path, &data);
    if (ok) printf("Wrote pool data to %s\n", pool_path);
    free_pool_data(&data);
    free_image(&image);
    return ok;
}

static int run_sort(const char *pool_path) {
    PoolData data;
    if (!read_pools(pool_path, &data)) return 0;
    print_sorted_pools(&data);
    free_pool_data(&data);
    return 1;
}

static int run_route(Point start, double fuel, const char *bmp_path, const char *pool_path,
                     const char *route_path, const char *overlay_path) {
    PoolData data;
    SearchContext context;
    FILE *improvements = NULL;
    int ok = 0;
    if (!read_pools(pool_path, &data)) return 0;
    if (start.x < 1 || start.x > data.width || start.y < 1 || start.y > data.height || fuel < 0.0) {
        fprintf(stderr, "Error: start point or fuel is outside the valid range.\n");
        free_pool_data(&data);
        return 0;
    }
    memset(&context, 0, sizeof(context));
    context.data = &data;
    context.destination = (Point){data.width, data.height};
    context.visited = calloc(data.count ? data.count : 1u, 1);
    if (!context.visited || !route_push(&context.current, (RouteStop){start, 0, 0.0, fuel})) {
        goto cleanup;
    }
    improvements = tmpfile();
    context.improvements = improvements;
    printf("Improving feasible routes:\n");
    if (!search_routes(&context)) goto cleanup;
    if (context.best.count == 0) {
        fprintf(stderr, "No feasible route reaches the destination.\n");
        goto cleanup;
    }
    if (!write_route(route_path, &context.best, data.width, data.height) ||
        !draw_route_overlay(bmp_path, overlay_path, &context.best,
                            (Pixel){37, 99, 235}, (Pixel){250, 204, 21})) {
        goto cleanup;
    }
    printf("Best route time: %.2f s\n", context.best.total_time);
    printf("Wrote %s and %s\n", route_path, overlay_path);
    ok = 1;
cleanup:
    if (improvements) fclose(improvements);
    free(context.visited);
    free(context.current.stops);
    free(context.best.stops);
    free_pool_data(&data);
    return ok;
}

static double euler_step(double cost, double action_cost, double dx) {
    return cost + dx * (COST_ALPHA / (cost + 1.0) + action_cost);
}

static int run_cost(double interval, const char *route_path, const char *csv_path) {
    Route route;
    int width, height;
    FILE *csv;
    double cost = 0.0;
    double distance = 0.0;
    double next_report = interval;
    if (interval <= 0.0 || !read_route(route_path, &route, &width, &height)) return 0;
    csv = fopen(csv_path, "w");
    if (!csv) {
        fprintf(stderr, "Error: cannot create cost data file '%s'.\n", csv_path);
        free(route.stops);
        return 0;
    }
    fprintf(csv, "distance_m,cost,phase\n0.000000,0.000000,start\n");
    printf("Distance (m)   Cost\n");
    printf("------------   --------\n");
    printf("%12.2f   %8.3f\n", distance, cost);
    for (size_t segment = 1; segment < route.count; ++segment) {
        double remaining = point_distance(route.stops[segment - 1].point, route.stops[segment].point);
        while (remaining > 1e-12) {
            double dx = remaining < EULER_STEP ? remaining : EULER_STEP;
            cost = euler_step(cost, 1.0, dx);
            distance += dx;
            remaining -= dx;
            fprintf(csv, "%.6f,%.9f,movement\n", distance, cost);
            while (distance + 1e-9 >= next_report) {
                printf("%12.2f   %8.3f\n", next_report, cost);
                next_report += interval;
            }
        }
        if (segment + 1 < route.count) {
            cost = euler_step(cost, 20.0, EULER_STEP);
            fprintf(csv, "%.6f,%.9f,extraction\n", distance, cost);
        }
    }
    printf("%12.2f   %8.3f (final)\n", distance, cost);
    fprintf(csv, "%.6f,%.9f,final\n", distance, cost);
    fclose(csv);
    free(route.stops);
    printf("Wrote cost profile to %s\n", csv_path);
    return 1;
}

static int run_fishing(int requested, const char *bmp_path, const char *pool_path,
                       const char *overlay_path) {
    PoolData data;
    Route route = {0};
    unsigned char *visited = NULL;
    int capacity = 0;
    int collected = 0;
    Point logistic = {0, 0};
    Point current;
    double distance = 0.0;
    int ok = 0;
    if (!read_pools(pool_path, &data)) return 0;
    for (size_t i = 0; i < data.count; ++i) {
        capacity += data.items[i].size;
        logistic.x += data.items[i].center.x;
        logistic.y += data.items[i].center.y;
    }
    if (data.count == 0 || requested < 1 || requested > capacity) {
        fprintf(stderr, "Error: fish order must be in the range 1-%d.\n", capacity);
        goto cleanup;
    }
    logistic.x = (int)lround((double)logistic.x / (double)data.count);
    logistic.y = (int)lround((double)logistic.y / (double)data.count);
    visited = calloc(data.count, 1);
    if (!visited || !route_push(&route, (RouteStop){logistic, 0, 0.0, 0.0})) goto cleanup;
    current = logistic;
    while (collected < requested) {
        int best = -1;
        double best_distance = HUGE_VAL;
        for (size_t i = 0; i < data.count; ++i) {
            double candidate;
            if (visited[i]) continue;
            candidate = point_distance(current, data.items[i].center);
            if (candidate < best_distance) {
                best_distance = candidate;
                best = (int)i;
            }
        }
        if (best < 0) goto cleanup;
        distance += best_distance;
        current = data.items[best].center;
        collected += data.items[best].size;
        visited[best] = 1;
        if (!route_push(&route, (RouteStop){current, data.items[best].size, 0.0, 0.0})) goto cleanup;
    }
    distance += point_distance(current, logistic);
    if (!route_push(&route, (RouteStop){logistic, 0, 0.0, 0.0}) ||
        !draw_route_overlay(bmp_path, overlay_path, &route,
                            (Pixel){234, 179, 8}, (Pixel){219, 39, 119})) goto cleanup;
    double fish_price = requested >= 100 ? requested * 0.75 : requested;
    double fuel_price = distance * 0.2;
    printf("Fishing order summary\n");
    printf("  Logistic point: (%d,%d)\n", logistic.x, logistic.y);
    printf("  Requested fish: %d\n", requested);
    printf("  Visited capacity: %d\n", collected);
    printf("  Round-trip distance: %.3f m\n", distance);
    printf("  Fish price: %.3f\n", fish_price);
    printf("  Fuel price: %.3f\n", fuel_price);
    printf("  Total price: %.3f\n", fish_price + fuel_price);
    printf("Wrote fishing route to %s\n", overlay_path);
    ok = 1;
cleanup:
    free(visited);
    free(route.stops);
    free_pool_data(&data);
    return ok;
}

static void print_usage(const char *program) {
    printf("Usage:\n");
    printf("  %s --scan [bmp] [pools-output]\n", program);
    printf("  %s --sort [pools]\n", program);
    printf("  %s --route X,Y FUEL [bmp] [pools] [route-output] [overlay-output]\n", program);
    printf("  %s --cost INTERVAL [route] [csv-output]\n", program);
    printf("  %s --fish COUNT [bmp] [pools] [overlay-output]\n", program);
    printf("  %s --help\n", program);
    printf("Run without arguments for the interactive menu.\n");
}

static int interactive_menu(void) {
    char line[128];
    for (;;) {
        int choice;
        printf("\nEscape Project\n");
        printf("1. Scan pools\n2. Print sorted pool list\n3. Plan escape route\n");
        printf("4. Compute numerical cost\n5. Plan fishing route\n9. Exit\nChoice: ");
        if (!fgets(line, sizeof(line), stdin) || !parse_int(line, &choice)) {
            fprintf(stderr, "Invalid menu choice.\n");
            continue;
        }
        if (choice == 1) {
            run_scan(DEFAULT_BMP, DEFAULT_POOLS);
        } else if (choice == 2) {
            run_sort(DEFAULT_POOLS);
        } else if (choice == 3) {
            Point start;
            double fuel;
            printf("Start coordinate (x,y): ");
            if (!fgets(line, sizeof(line), stdin) || !parse_point(line, &start)) continue;
            printf("Initial fuel (cm^3): ");
            if (!fgets(line, sizeof(line), stdin) || !parse_double(line, &fuel)) continue;
            run_route(start, fuel, DEFAULT_BMP, DEFAULT_POOLS, DEFAULT_ROUTE, DEFAULT_ROUTE_BMP);
        } else if (choice == 4) {
            double interval;
            printf("Display interval in meters: ");
            if (!fgets(line, sizeof(line), stdin) || !parse_double(line, &interval)) continue;
            run_cost(interval, DEFAULT_ROUTE, DEFAULT_COST_CSV);
        } else if (choice == 5) {
            int fish;
            printf("Requested fish: ");
            if (!fgets(line, sizeof(line), stdin) || !parse_int(line, &fish)) continue;
            run_fishing(fish, DEFAULT_BMP, DEFAULT_POOLS, DEFAULT_FISH_BMP);
        } else if (choice == 9) {
            return 0;
        } else {
            fprintf(stderr, "Invalid menu choice.\n");
        }
    }
}

int main(int argc, char **argv) {
    if (argc == 1) return interactive_menu();
    if (strcmp(argv[1], "--help") == 0) {
        print_usage(argv[0]);
        return 0;
    }
    if (strcmp(argv[1], "--scan") == 0) {
        return run_scan(argc > 2 ? argv[2] : DEFAULT_BMP,
                        argc > 3 ? argv[3] : DEFAULT_POOLS) ? 0 : 1;
    }
    if (strcmp(argv[1], "--sort") == 0) {
        return run_sort(argc > 2 ? argv[2] : DEFAULT_POOLS) ? 0 : 1;
    }
    if (strcmp(argv[1], "--route") == 0 && argc >= 4) {
        Point start;
        double fuel;
        if (!parse_point(argv[2], &start) || !parse_double(argv[3], &fuel)) {
            print_usage(argv[0]);
            return 2;
        }
        return run_route(start, fuel, argc > 4 ? argv[4] : DEFAULT_BMP,
                         argc > 5 ? argv[5] : DEFAULT_POOLS,
                         argc > 6 ? argv[6] : DEFAULT_ROUTE,
                         argc > 7 ? argv[7] : DEFAULT_ROUTE_BMP) ? 0 : 1;
    }
    if (strcmp(argv[1], "--cost") == 0 && argc >= 3) {
        double interval;
        if (!parse_double(argv[2], &interval)) return 2;
        return run_cost(interval, argc > 3 ? argv[3] : DEFAULT_ROUTE,
                        argc > 4 ? argv[4] : DEFAULT_COST_CSV) ? 0 : 1;
    }
    if (strcmp(argv[1], "--fish") == 0 && argc >= 3) {
        int fish;
        if (!parse_int(argv[2], &fish)) return 2;
        return run_fishing(fish, argc > 3 ? argv[3] : DEFAULT_BMP,
                           argc > 4 ? argv[4] : DEFAULT_POOLS,
                           argc > 5 ? argv[5] : DEFAULT_FISH_BMP) ? 0 : 1;
    }
    print_usage(argv[0]);
    return 2;
}
