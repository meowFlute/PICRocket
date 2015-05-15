#define BUFFER_SIZE 4096

typedef struct {
    unsigned char jmp[3];
    char oem[8];
    unsigned char sector_size[2];
    unsigned char sectors_per_cluster;
    unsigned char reserved_sectors[2];
    unsigned char number_of_fats;
    unsigned char root_dir_entries[2];
    unsigned char total_sectors_short[2]; // if zero, later field is used
    unsigned char media_descriptor;
    unsigned char fat_size_sectors[2];
    unsigned char sectors_per_track[2];
    unsigned char number_of_heads[2];
    unsigned char hidden_sectors[4];
    unsigned char total_sectors_long[4];   
    unsigned char drive_number;
    unsigned char current_head;
    unsigned char boot_signature;
    unsigned char volume_id[4];
    char volume_label[11];
    char fs_type[8];
    char boot_code[448];
    unsigned char boot_sector_signature[2];
}Fat16BootSector;

typedef struct {
    unsigned char filename[8];
    unsigned char ext[3];
    unsigned char attributes;
    unsigned char reserved[10];
    unsigned char modify_time[2];
    unsigned char modify_date[2];
    unsigned char starting_cluster[2];
    unsigned char file_size[4];
}Fat16Entry;

typedef struct {
    unsigned char first_byte;
    unsigned char start_chs[3];
    unsigned char partition_type;
    unsigned char end_chs[3];
    unsigned long start_sector;
    unsigned long length_sectors;
}PartitionTable;

void print_file_info(Fat16Entry *entry);
int print_partition_table(PartitionTable pt[]);
void print_boot_sector(Fat16BootSector bs);
void fat_read_file(FILE * in, FILE * out,
                   unsigned long fat_start, 
                   unsigned long data_start, 
                   unsigned long cluster_size, 
                   unsigned short cluster, 
                   unsigned long file_size);
unsigned short fat_fopen_writefile(unsigned long fat_start, FILE * in, unsigned long cluster_size);
void fat_write(FILE * in, 
			   unsigned char buffer[BUFFER_SIZE], 
			   unsigned short *cluster, 
			   unsigned long *cluster_left, 
			   unsigned short length,  
			   unsigned long *address, 
			   unsigned long fat_start, 
			   unsigned long data_start, 
               unsigned long cluster_size);
void fat_fclose_writefile(Fat16Entry *named_entry);