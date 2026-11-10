#include "mapping_grid.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>
void MappingGrid_Clear(MappingGrid *g) { if(g) memset(g,0,sizeof(*g)); }
static void Grid_Cell(MappingGrid *g,float x,float y,uint8_t state) {
    float col=floorf((x-(float)MAP_ORIGIN_X_MM)/MAP_CELL_MM);
    float row=floorf((y-(float)MAP_ORIGIN_Y_MM)/MAP_CELL_MM);
    if(!isfinite(col)||!isfinite(row)||col<0||row<0||col>=MAP_WIDTH||row>=MAP_HEIGHT) return;
    uint8_t *cell=&g->cells[(unsigned)row*MAP_WIDTH+(unsigned)col];
    /* Conservador: espacio libre no borra una hipotesis de ocupacion previa. */
    if(state>*cell) { *cell=state;++g->revision; }
}
void MappingGrid_AddPose(MappingGrid *g,int32_t x,int32_t y) {
    if(!g) return;
    if(g->path_count) {
        unsigned last=(g->path_next+MAP_PATH_LENGTH-1U)%MAP_PATH_LENGTH;
        if(llabs((long long)x-g->path[last][0])+llabs((long long)y-g->path[last][1])<MAP_CELL_MM/2U) return;
    }
    g->path[g->path_next][0]=x;g->path[g->path_next][1]=y;
    g->path_next=(g->path_next+1U)%MAP_PATH_LENGTH;
    if(g->path_count<MAP_PATH_LENGTH) ++g->path_count;
}
bool MappingGrid_Ray(MappingGrid *g,int32_t x,int32_t y,int32_t yaw,uint32_t range,uint32_t beam,uint32_t age) {
    if(!g||x< -101000||x>101000||y< -101000||y>101000||yaw< -3142||yaw>3142||
       range<20U||range>4000U||beam<50U||beam>785U||age>MAP_MEASUREMENT_MAX_AGE_MS) return false;
    const float a=yaw/1000.0f,cs=cosf(a),sn=sinf(a);
    /* Solo la linea central aporta evidencia libre; no vaciar un cono entero.
     * Anillo angular de incertidumbre para UN eco, no puntos de un lidar. */
    unsigned step=MAP_CELL_MM/2U;
    for(unsigned d=0;d+MAP_RANGE_UNCERTAINTY_MM<range;d+=step)
        Grid_Cell(g,x+d*cs,y+d*sn,1U);
    unsigned segments=(unsigned)ceilf(2.0f*(beam/1000.0f)*range/MAP_CELL_MM);
    if(segments<1U) segments=1U;
    if(segments>320U) segments=320U;
    for(unsigned i=0;i<=segments;++i) {
        float angle=a+(beam/1000.0f)*(2.0f*i/segments-1.0f);
        float c=cosf(angle),s=sinf(angle);
        for(int offset=-(int)MAP_RANGE_UNCERTAINTY_MM;offset<=(int)MAP_RANGE_UNCERTAINTY_MM;offset+=(int)MAP_CELL_MM) {
            float d=(float)range+offset;
            if(d>0) Grid_Cell(g,x+d*c,y+d*s,2U);
        }
    }
    return true;
}
