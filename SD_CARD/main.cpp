#include <stdio.h>
#include <stdlib.h>
#include <memory.h>
#include "functions.h"

int main() {
    FILE * in = fopen("test3.img", "rb");
	FILE * out;
    int i;
    //PartitionTable pt[4];
    Fat16BootSector bs;
	Fat16Entry entry;
	char filename[] = "SD-SPI  "; //must be 8 chars
	char file_ext[] = "C  "; //must be 3 chars
    
	//*************************************** read the partition table *********************************
	//****this disk has no partition table
    //fseek(in, 0x1BE, SEEK_SET); // go to partition table start
    //fread(pt, sizeof(PartitionTable), 4, in); // read all four entries
    //print_partition_table(pt);

	//*************************************** read the boot sector *************************************
    fseek(in, 512 * 0, SEEK_SET);
    fread(&bs, sizeof(Fat16BootSector), 1, in);
    print_boot_sector(bs);
	//convert data types (needed to be chars for packing purposes... could change later?)
	unsigned short sector_size = *(bs.sector_size+1) << 8 | *(bs.sector_size+0);
	unsigned short reserved_sectors = *(bs.reserved_sectors+1) << 8 | *(bs.reserved_sectors+0);
	unsigned short root_dir_entries = *(bs.root_dir_entries+1) << 8 | *(bs.root_dir_entries+0);
	unsigned short total_sectors_short = *(bs.total_sectors_short+1) << 8 | *(bs.total_sectors_short+0);
	unsigned short fat_size_sectors = *(bs.fat_size_sectors+1) << 8 | *(bs.fat_size_sectors+0);
	unsigned short sectors_per_track = *(bs.sectors_per_track+1) << 8 | *(bs.sectors_per_track+0);
	unsigned short number_of_heads = *(bs.number_of_heads+1) << 8 | *(bs.number_of_heads+0);
	unsigned long hidden_sectors = *(bs.hidden_sectors+3) << 8*3 | *(bs.hidden_sectors+2) << 8*2 | *(bs.hidden_sectors+1) << 8 | *(bs.hidden_sectors+0);
	unsigned long total_sectors_long = *(bs.total_sectors_long+3) << 8*3 | *(bs.total_sectors_long+2) << 8*2 | *(bs.total_sectors_long+1) << 8 | *(bs.total_sectors_long+0);
	unsigned short boot_sector_signature = *(bs.boot_sector_signature+1) << 8 | *(bs.boot_sector_signature+0);

	 printf("Now at 0x%X, sector size %d, FAT size %d sectors, %d FATs\n", ftell(in), sector_size, fat_size_sectors, bs.number_of_fats);

	//*************************************** Find and read the root directory *************************
	// Calculate start offsets of FAT, root directory and data
    unsigned long fat_start = ftell(in) + (reserved_sectors-1) * sector_size;
	unsigned long root_start = fat_start + fat_size_sectors * bs.number_of_fats * sector_size;
    unsigned long data_start = root_start + root_dir_entries * sizeof(Fat16Entry);
    printf("FAT start at 0x%X, root dir at 0x%X, data at 0x%X\n", fat_start, root_start, data_start);
	printf("Jumping %d sectors to 0x%X which is the root directory...\n\n", (reserved_sectors-1 + fat_size_sectors * bs.number_of_fats),((reserved_sectors-1 + fat_size_sectors * bs.number_of_fats) *sector_size)+ftell(in));

	fseek(in, root_start, SEEK_SET);
	printf("*****ROOT DIRECTORY*****\n");
	for(i=0; i<root_dir_entries; i++) 
	{
		fread(&entry, sizeof(entry), 1, in);
		print_file_info(&entry);
		if(memcmp(entry.filename, filename, 8) == 0 && memcmp(entry.ext, file_ext, 3) == 0) 
		{
            printf("File found!\n");
			i = -1;
            break;
        }
	}
	if(i == -1)
	{
		printf("File [%s.%s] found!\n", filename, file_ext);
	}
	else 
	{
		printf("\nRoot directory read and file [%s.%s] not found.\nNow at 0x%X\n", filename, file_ext, ftell(in));
	}
	unsigned short modify_time = *(entry.modify_time+1) << 8 | *(entry.modify_time+0);
	unsigned short modify_date = *(entry.modify_date+1) << 8 | *(entry.modify_date+0);
	unsigned short starting_cluster = *(entry.starting_cluster+1) << 8 | *(entry.starting_cluster+0);
	unsigned long file_size = *(entry.file_size+3) << 8*3 | *(entry.file_size+2) << 8*2 | *(entry.file_size+1) << 8 | *(entry.file_size+0);

	//*************************************** Read a specific file from disk and save it ***************
	out = fopen("SD-SPI.c", "wb"); //output file
	fat_read_file(in, out, fat_start, data_start, bs.sectors_per_cluster * sector_size, starting_cluster, file_size);

	//*************************************** Close the image file and pause ***************************
    fclose(in);
    system("pause");
	return 0;
}

