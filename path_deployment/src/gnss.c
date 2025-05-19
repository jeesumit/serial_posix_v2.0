#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h> 

int main() {
    FILE *file;

    srand((unsigned int)time(NULL));

    while (1) {
        file = fopen("../data/gnss.buf", "a"); 
        if (file == NULL) {
            printf("Error opening file!\n");
            return 1;
        }

        double lat = 28+(double)rand() / RAND_MAX;
        double lon = 77+ (double)rand() / RAND_MAX;
        double latk = 28+(double)rand() / RAND_MAX;
        double lonk = 77+ (double)rand() / RAND_MAX;
        // Write data to file
        fprintf(file, "Lat:%.4f Lng:%.4f Latk:%.4f Lngk:%.4f \n", lat, lon, latk , lonk);
        fclose(file);

        //printf("Wrote: Time: 12345 Date:234567 speed:0.34 Lat: %.4f Lng %.4f head:12.34 hdop:1.23 height:3.45\n", lat, lon);

        //usleep(500000);
    }

    return 0;
}

