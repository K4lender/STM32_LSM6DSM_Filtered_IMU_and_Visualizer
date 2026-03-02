# Makefile - IMU Simulasyon Projesi
# Windows: mingw32-make veya make (Git Bash)
# Linux:   make

# Derleyici
CC = gcc

# Dizinler
SRC_DIR = src
INC_DIR = inc

# Kaynak dosyalari
SOURCES = $(SRC_DIR)/main.c \
          $(SRC_DIR)/lsm6dsm_driver.c \
		  $(SRC_DIR)/lsm6dsm_process.c \
          $(SRC_DIR)/filter.c \
          $(SRC_DIR)/ring_buffer.c \
          $(SRC_DIR)/timer.c \
          $(SRC_DIR)/imu_sim.c

# Cikti dosyasi
ifeq ($(OS),Windows_NT)
    TARGET = imu_sim.exe
    RM = del /Q
else
    TARGET = imu_sim
    RM = rm -f
endif

# Derleme bayraklari
CFLAGS = -I$(INC_DIR) -Wall -Wextra -O2

# Linux icin matematik kutuphanesi
ifeq ($(OS),Windows_NT)
    LDFLAGS =
else
    LDFLAGS = -lm
endif

# Varsayilan hedef
all: $(TARGET)

# Derleme
$(TARGET): $(SOURCES)
	$(CC) $(CFLAGS) -o $(TARGET) $(SOURCES) $(LDFLAGS)
	@echo ""
	@echo "Derleme basarili: $(TARGET)"
	@echo ""
	@echo "Kullanim: imu_sim.exe [senaryo_no]"
	@echo "  0 veya bos: Tum senaryolari calistir"
	@echo "  1: Normal calisma"
	@echo "  2: Veri kaybi testi"
	@echo "  3: Spike + 50 Hz EMI filtreleme testi"
	@echo "  4: Gecikmeli veri testi"

# Calistir
run: $(TARGET)
	./$(TARGET)

# Temizlik
clean:
	$(RM) $(TARGET)

# Debug derlemesi
debug: CFLAGS += -g -DDEBUG
debug: $(TARGET)

# Yardim
help:
	@echo "Kullanim:"
	@echo "  make        - Projeyi derle"
	@echo "  make run    - Derle ve calistir"
	@echo "  make clean  - Temizle"
	@echo "  make debug  - Debug modunda derle"

.PHONY: all run clean debug help
