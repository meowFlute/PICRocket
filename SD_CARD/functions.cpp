#include <stdlib.h>
#include <stdio.h>
#include "functions.h"


void print_file_info(Fat16Entry *entry) 
{
    switch(entry->filename[0]) {
    case 0x00:
        return; // unused entry
    case 0xE5:
        printf("Deleted file: [?%.7s.%.3s]\n", entry->filename+1, entry->ext);
        return;
    case 0x05:
        printf("File starting with 0xE5: [%c%.7s.%.3s]\n", 0xE5, entry->filename+1, entry->ext);
        break;
    case 0x2E:
        printf("Directory: [%.8s.%.3s]\n", entry->filename, entry->ext);
        break;
    default:
        printf("File: [%.8s.%.3s]\n", entry->filename, entry->ext);
    }
    
	unsigned short modify_date = *(entry->modify_date+1) << 8 | *(entry->modify_date+0);
	unsigned short modify_time = *(entry->modify_time+1) << 8 | *(entry->modify_time+0);
	unsigned short starting_cluster = *(entry->starting_cluster+1) << 8 | *(entry->starting_cluster+0);
	unsigned long file_size = *(entry->file_size+3) << 8*3 | *(entry->file_size+2) << 8*2 | *(entry->file_size+1) << 8 | *(entry->file_size+0);
    printf("  Modified: %04d-%02d-%02d %02d:%02d.%02d    Start: [%04X]    Size: %d\n\n", 
        1980 + (modify_date >> 9), (modify_date >> 5) & 0xF, modify_date & 0x1F,
        (modify_time >> 11), (modify_time >> 5) & 0x3F, modify_time & 0x1F,
        starting_cluster, file_size);
}

void print_boot_sector(Fat16BootSector bs)
{
	printf("*******BOOT SECTOR DATA*******\n");
    printf("  Jump code: %02X:%02X:%02X\n", bs.jmp[0], bs.jmp[1], bs.jmp[2]);

    printf("  OEM code: [%.8s]\n", bs.oem);

	unsigned short sector_size = *(bs.sector_size+1) << 8 | *(bs.sector_size+0);
    printf("  sector_size: %hu\n", sector_size);

    printf("  sectors_per_cluster: %d\n", bs.sectors_per_cluster);

	unsigned short reserved_sectors = *(bs.reserved_sectors+1) << 8 | *(bs.reserved_sectors+0);
    printf("  reserved_sectors: %d\n", reserved_sectors);

    printf("  number_of_fats: %d\n", bs.number_of_fats);

	unsigned short root_dir_entries = *(bs.root_dir_entries+1) << 8 | *(bs.root_dir_entries+0);
    printf("  root_dir_entries: %d\n", root_dir_entries);

	unsigned short total_sectors_short = *(bs.total_sectors_short+1) << 8 | *(bs.total_sectors_short+0);
    printf("  total_sectors_short: %d\n", total_sectors_short);

    printf("  media_descriptor: 0x%02X\n", bs.media_descriptor);

	unsigned short fat_size_sectors = *(bs.fat_size_sectors+1) << 8 | *(bs.fat_size_sectors+0);
    printf("  fat_size_sectors: %d\n", fat_size_sectors);

	unsigned short sectors_per_track = *(bs.sectors_per_track+1) << 8 | *(bs.sectors_per_track+0);
    printf("  sectors_per_track: %d\n", sectors_per_track);

	unsigned short number_of_heads = *(bs.number_of_heads+1) << 8 | *(bs.number_of_heads+0);
    printf("  number_of_heads: %d\n", number_of_heads);

	unsigned long hidden_sectors = *(bs.hidden_sectors+3) << 8*3 | *(bs.hidden_sectors+2) << 8*2 | *(bs.hidden_sectors+1) << 8 | *(bs.hidden_sectors+0);
    printf("  hidden_sectors: %d\n", hidden_sectors);

	unsigned long total_sectors_long = *(bs.total_sectors_long+3) << 8*3 | *(bs.total_sectors_long+2) << 8*2 | *(bs.total_sectors_long+1) << 8 | *(bs.total_sectors_long+0);
    printf("  total_sectors_long: %d\n", total_sectors_long);

    printf("  drive_number: 0x%02X\n", bs.drive_number);

    printf("  current_head: 0x%02X\n", bs.current_head);

    printf("  boot_signature: 0x%02X\n", bs.boot_signature);

    printf("  volume_id: 0x%08X\n", bs.volume_id);

    printf("  Volume label: [%.11s]\n", bs.volume_label);

    printf("  Filesystem type: [%.8s]\n", bs.fs_type);

	unsigned short boot_sector_signature = *(bs.boot_sector_signature+1) << 8 | *(bs.boot_sector_signature+0);
    printf("  Boot sector signature: 0x%04X\n\n\n", boot_sector_signature);
}

int print_partition_table(PartitionTable pt[])
{
	int i;
    
    for(i=0; i<4; i++) {        
        if(pt[i].partition_type == 4 || pt[i].partition_type == 6 ||
           pt[i].partition_type == 14) {
            printf("FAT16 filesystem found from partition %d\n", i);
            return 0;
        }
    }
    
    if(i == 4) {
        printf("No FAT16 filesystem found, exiting...\n");
        return -1;
    }
}

void fat_read_file(FILE * in, FILE * out,
                   unsigned long fat_start, 
                   unsigned long data_start, 
                   unsigned long cluster_size, 
                   unsigned short cluster, 
                   unsigned long file_size) {
    unsigned char buffer[4096];
    size_t bytes_read, bytes_to_read,
           file_left = file_size, cluster_left = cluster_size;

    // Go to first data cluster
	printf("\nThe first cluster is 0x%02X which is cluster #%d\n", cluster, cluster-2);
	printf("The data starts at 0x%X and the cluster size is 0x%X\n", data_start, cluster_size);
	printf("Jumping to first cluster: 0x%X + (0x%X * %d) = 0x%X\n", data_start, cluster_size, cluster-2, data_start+cluster_size*(cluster-2) );
	printf("There are %d bytes left to read in the file\n\n", file_left);
    fseek(in, data_start + cluster_size * (cluster-2), SEEK_SET);
    
    // Read until we run out of file or clusters
    while(file_left > 0 && cluster != 0xFFFF) {
        bytes_to_read = sizeof(buffer);
        
        // don't read past the file or cluster end
        if(bytes_to_read > file_left)
            bytes_to_read = file_left;
        if(bytes_to_read > cluster_left)
            bytes_to_read = cluster_left;
        
        // read data from cluster, write to file
        bytes_read = fread(buffer, 1, bytes_to_read, in);
        fwrite(buffer, 1, bytes_read, out);
		//printf("%s", buffer); //out to console
        printf("Copied %d bytes\n", bytes_read); //out to file
        
        // decrease byte counters for current cluster and whole file
        cluster_left -= bytes_read;
        file_left -= bytes_read;
        
        // if we have read the whole cluster, read next cluster # from FAT
        if(cluster_left == 0) {
            fseek(in, fat_start + cluster*2, SEEK_SET);

			printf("End of cluster reached\n\nJumping to last FAT (File Allocation Table) location at 0x%X\n", fat_start+cluster*2 );

            fread(&cluster, 2, 1, in);
            
            printf("Reading two bytes... next cluster read as 0x%02X which is %d\n", cluster, cluster);
            printf("Jumping to next cluster: 0x%X + (0x%X * %d) = 0x%X\n", data_start, cluster_size, cluster-2, data_start+cluster_size*(cluster-2) );
			printf("There are %d bytes left to read in the file\n\n", file_left);

            fseek(in, data_start + cluster_size * (cluster-2), SEEK_SET);
            cluster_left = cluster_size; // reset cluster byte counter
        }
    }
}