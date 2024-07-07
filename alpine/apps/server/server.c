#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define DIR "/sys/bus/iio/devices/iio:device0/"
#define EEPROM "/sys/bus/i2c/devices/0-0050/eeprom"

const char *directory = "/media/mmcblk0p1/apps";
const char *forbidden = "HTTP/1.0 403 Forbidden\n\n";
const char *redirect = "HTTP/1.0 302 Found\nLocation: /\n\n";
const char *okheader = "HTTP/1.0 200 OK\n\n";

void detach(char *path)
{
  int pid = fork();
  if(pid != 0) return;
  close(STDIN_FILENO);
  close(STDOUT_FILENO);
  close(STDERR_FILENO);
  execlp(path, path, NULL);
  exit(0);
}

float read_value(char *name)
{
  FILE *fp;
  char buffer[64];

  if((fp = fopen(name, "r")) == NULL)
  {
    printf("Cannot open %s.\n", name);
    exit(1);
  }

  fgets(buffer, sizeof(buffer), fp);
  fclose(fp);

  return atof(buffer);
}

char *read_string(char *name, char *value, size_t value_size)
{
  FILE *fp;
  size_t len;

  if((fp = fopen(name, "r")) == NULL)
  {
    printf("Cannot open %s.\n", name);
    exit(1);
  }

  fgets(value, value_size, fp);
  fclose(fp);

  /* remove CR/LF */
  len = strlen(value);
  if(len > 0 && value[len-1] == '\n')
    value[len-1] = '\0';
  len = strlen(value);
  if(len > 0 && value[len-1] == '\r')
    value[len-1] = '\0';

  return value;
}

float read_temp0()
{
  float off, raw, scl;
  off = read_value(DIR "in_temp0_offset");
  raw = read_value(DIR "in_temp0_raw");
  scl = read_value(DIR "in_temp0_scale");
  return (off + raw) * scl / 1000;
}

float read_volt(char *name, char *label, size_t label_size)
{
  float raw, scl;
  char varname[256];
  snprintf(varname, 255, "%s%s_raw", DIR, name);
  raw = read_value(varname);
  snprintf(varname, 255, "%s%s_scale", DIR, name);
  scl = read_value(varname);
  snprintf(varname, 255, "%s%s_label", DIR, name);
  read_string(varname, label, label_size);
  return raw * scl / 1000;
}

int main(int argc, char *argv[])
{
  FILE *fp;
  int fd, id, i, j, k, top;
  float temp0, volt;
  struct stat sb;
  size_t size;
  char buffer[256];
  char path[291];
  char label[64];
  char eeprom[1024];
  char *end;
  long freq;
  volatile int *slcr;

  if((fd = open("/dev/mem", O_RDWR)) < 0)
  {
    fwrite(forbidden, 24, 1, stdout);
    return 1;
  }

  slcr = mmap(NULL, sysconf(_SC_PAGESIZE), PROT_READ|PROT_WRITE, MAP_SHARED, fd, 0xF8000000);
  id = (slcr[332] >> 12) & 0x1f;

  freq = (argc == 2) ? strtol(argv[1], &end, 10) : -1;
  if(errno != 0 || end == argv[1] || freq < 0)
  {
    freq = 125;
  }

  if(fgets(buffer, 256, stdin) == NULL)
  {
    fwrite(forbidden, 24, 1, stdout);
    return 1;
  }

  if(buffer[4] != '/')
  {
    fwrite(forbidden, 24, 1, stdout);
    return 1;
  }

  if(strncmp(buffer, "GET ", 4) && strncmp(buffer, "get ", 4))
  {
    fwrite(forbidden, 24, 1, stdout);
    return 1;
  }

  top = 1;
  for(i = 5; i < 255; ++i)
  {
    if(buffer[i] == ' ')
    {
      buffer[i] = 0;
      break;
    }
    if(buffer[i] != '/') top = 0;
  }

  for(j = 5; j < i - 1; ++j)
  {
    if(buffer[j] == '.' && buffer[j + 1] == '.')
    {
      fwrite(forbidden, 24, 1, stdout);
      return 1;
    }
  }

  if(i == 10 && strncmp(buffer + 5, "temp0", 5) == 0)
  {
    fwrite(okheader, 17, 1, stdout);
    temp0 = read_temp0();
    printf("%.1f\n", temp0);
    return 0;
  }

  if(i == 12 && strncmp(buffer + 5, "sensors", 7) == 0)
  {
    fwrite(okheader, 17, 1, stdout);
    temp0 = read_temp0();
    printf("%s=%.1f\n", "temp0", temp0);
    volt = read_volt("in_voltage0_vccint", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage1_vccaux", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage2_vccbram", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage3_vccpint", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage4_vccpaux", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage5_vccoddr", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage6_vrefp", label, 64);
    printf("%s=%.3f\n", label, volt);
    volt = read_volt("in_voltage7_vrefn", label, 64);
    printf("%s=%.3f\n", label, volt);
    return 0;
  }

  if(i == 11 && strncmp(buffer + 5, "eeprom", 6) == 0)
  {
    if((fd = open(EEPROM, O_RDONLY)) < 0)
    {
      fwrite(forbidden, 24, 1, stdout);
      return 1;
    }
    if(lseek(fd, 0x1800, SEEK_SET) < 0)
    {
      fwrite(forbidden, 24, 1, stdout);
      return 1;
    }
    if(read(fd, eeprom, 1024) < 0)
    {
      fwrite(forbidden, 24, 1, stdout);
      return 1;
    }
    /* replace '\0' with '\n' */
    for(k = 4; k < 1024; ++k)
    {
      if(eeprom[k] == 0)
      {
        eeprom[k] = '\n';
        if(k < 1023 && eeprom[k+1] == 0)
        {
          break;
        }
      }
    }
    fwrite(okheader, 17, 1, stdout);
    fwrite(eeprom + 4, k + 1 - 4, 1, stdout);
    return 0;
  }

  /* debug */
  if(i == 10 && strncmp(buffer + 5, "debug", 5) == 0)
  {
    fwrite(okheader, 17, 1, stdout);
    printf("id=%d\n", id);
    printf("freq=%ld\n", freq);
    return 0;
  }

  memcpy(path, directory, 21);
  memcpy(path + 21, buffer + 4, i - 3);

  if(stat(path, &sb) < 0)
  {
    fwrite(redirect, 32, 1, stdout);
    return 1;
  }

  if(S_ISDIR(sb.st_mode))
  {
    memcpy(path + 21 + i - 4, "/start.sh", 10);
    detach(path);
    if(top && id == 7 && freq == 122)
    {
      memcpy(path + 21 + i - 4, "/index_122_88.html", 19);
    }
    else if(top && id == 2 && freq == 125)
    {
      memcpy(path + 21 + i - 4, "/index_trx_duo.html", 20);
    }
    else
    {
      memcpy(path + 21 + i - 4, "/index.html", 12);
    }
  }

  fp = fopen(path, "r");

  if(fp == NULL)
  {
    fwrite(redirect, 32, 1, stdout);
    return 1;
  }

  fwrite(okheader, 17, 1, stdout);

  while((size = fread(buffer, 1, 256, fp)) > 0)
  {
    if(!fwrite(buffer, size, 1, stdout)) break;
  }

  return 0;
}
