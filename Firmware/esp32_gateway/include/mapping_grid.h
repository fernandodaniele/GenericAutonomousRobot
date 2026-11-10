#ifndef MAPPING_GRID_H
#define MAPPING_GRID_H
#include <stdbool.h>
#include <stdint.h>
#ifndef MAP_WIDTH
#define MAP_WIDTH 80U
#endif
#ifndef MAP_HEIGHT
#define MAP_HEIGHT 80U
#endif
#ifndef MAP_CELL_MM
#define MAP_CELL_MM 50U
#endif
#define MAP_CELLS (MAP_WIDTH*MAP_HEIGHT)
#define MAP_PATH_LENGTH 128U
#define MAP_ORIGIN_X_MM (-(int32_t)(MAP_WIDTH*MAP_CELL_MM/2U))
#define MAP_ORIGIN_Y_MM (-(int32_t)(MAP_HEIGHT*MAP_CELL_MM/2U))
#define MAP_MEASUREMENT_MAX_AGE_MS 300U
#define MAP_RANGE_UNCERTAINTY_MM 100U
#define MAP_JSON_CAPACITY (MAP_CELLS+MAP_PATH_LENGTH*32U+2048U)
_Static_assert(MAP_WIDTH>=8U&&MAP_WIDTH<=128U&&MAP_HEIGHT>=8U&&MAP_HEIGHT<=128U&&MAP_CELLS<=10000U&&MAP_CELL_MM>=20U&&MAP_CELL_MM<=200U,"mapping memory/config bound");
typedef struct {
    uint8_t cells[MAP_CELLS]; /* 0 desconocida, 1 libre observado, 2 ocupacion posible */
    int32_t path[MAP_PATH_LENGTH][2];
    unsigned path_next,path_count;
    uint32_t revision;
} MappingGrid;
void MappingGrid_Clear(MappingGrid *grid);
void MappingGrid_AddPose(MappingGrid *grid,int32_t x_mm,int32_t y_mm);
bool MappingGrid_Ray(MappingGrid *grid,int32_t x_mm,int32_t y_mm,int32_t yaw_mrad,
                      uint32_t distance_mm,uint32_t half_angle_mrad,uint32_t age_ms);
#endif
