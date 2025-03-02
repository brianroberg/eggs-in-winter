/*
 * Sunrise and sunset schedule
 * We will calculate our lighting schedule from this.
 */
const byte sunTimes[177][6] = {
  // month, day, rise hour, rise minute, set hour, set minute
  // always in standard time (no DST)
  {10, 24, 6, 32, 17, 18},
  {10, 25, 6, 33, 17, 16},
  {10, 26, 6, 34, 17, 15},
  {10, 27, 6, 35, 17, 14},
  {10, 28, 6, 37, 17, 12},
  {10, 29, 6, 38, 17, 11},
  {10, 30, 6, 39, 17, 10},
  {10, 31, 6, 40, 17, 9},
  {11, 1, 6, 41, 17, 7},
  {11, 2, 6, 42, 17, 6},
  {11, 3, 6, 44, 17, 5},
  {11, 4, 6, 45, 17, 4},
  {11, 5, 6, 46, 17, 3},
  {11, 6, 6, 47, 17, 2},
  {11, 7, 6, 48, 17, 1},
  {11, 8, 6, 49, 17, 0},
  {11, 9, 6, 51, 16, 59},
  {11, 10, 6, 52, 16, 58},
  {11, 11, 6, 53, 16, 57},
  {11, 12, 6, 54, 16, 56},
  {11, 13, 6, 55, 16, 55},
  {11, 14, 6, 57, 16, 54},
  {11, 15, 6, 58, 16, 53},
  {11, 16, 6, 59, 16, 52},
  {11, 17, 7, 0, 16, 51},
  {11, 18, 7, 1, 16, 51},
  {11, 19, 7, 2, 16, 50},
  {11, 20, 7, 4, 16, 49},
  {11, 21, 7, 5, 16, 49},
  {11, 22, 7, 6, 16, 48},
  {11, 23, 7, 7, 16, 48},
  {11, 24, 7, 8, 16, 47},
  {11, 25, 7, 9, 16, 46},
  {11, 26, 7, 10, 16, 46},
  {11, 27, 7, 11, 16, 46},
  {11, 28, 7, 12, 16, 45},
  {11, 29, 7, 13, 16, 45},
  {11, 30, 7, 15, 16, 45},
  {12, 1, 7, 16, 16, 44},
  {12, 2, 7, 17, 16, 44},
  {12, 3, 7, 18, 16, 44},
  {12, 4, 7, 19, 16, 44},
  {12, 5, 7, 20, 16, 44},
  {12, 6, 7, 20, 16, 43},
  {12, 7, 7, 21, 16, 43},
  {12, 8, 7, 22, 16, 43},
  {12, 9, 7, 23, 16, 43},
  {12, 10, 7, 24, 16, 44},
  {12, 11, 7, 25, 16, 44},
  {12, 12, 7, 26, 16, 44},
  {12, 13, 7, 26, 16, 44},
  {12, 14, 7, 27, 16, 44},
  {12, 15, 7, 28, 16, 44},
  {12, 16, 7, 29, 16, 45},
  {12, 17, 7, 29, 16, 45},
  {12, 18, 7, 30, 16, 45},
  {12, 19, 7, 30, 16, 46},
  {12, 20, 7, 31, 16, 46},
  {12, 21, 7, 31, 16, 47},
  {12, 22, 7, 32, 16, 47},
  {12, 23, 7, 32, 16, 48},
  {12, 24, 7, 33, 16, 48},
  {12, 25, 7, 33, 16, 49},
  {12, 26, 7, 34, 16, 50},
  {12, 27, 7, 34, 16, 50},
  {12, 28, 7, 34, 16, 51},
  {12, 29, 7, 34, 16, 52},
  {12, 30, 7, 35, 16, 53},
  {12, 31, 7, 35, 16, 53},
  {1, 1, 7, 35, 16, 54},
  {1, 2, 7, 35, 16, 55},
  {1, 3, 7, 35, 16, 56},
  {1, 4, 7, 35, 16, 57},
  {1, 5, 7, 35, 16, 58},
  {1, 6, 7, 35, 16, 59},
  {1, 7, 7, 35, 17, 0},
  {1, 8, 7, 35, 17, 1},
  {1, 9, 7, 35, 17, 2},
  {1, 10, 7, 34, 17, 3},
  {1, 11, 7, 34, 17, 4},
  {1, 12, 7, 34, 17, 5},
  {1, 13, 7, 34, 17, 6},
  {1, 14, 7, 33, 17, 7},
  {1, 15, 7, 33, 17, 8},
  {1, 16, 7, 32, 17, 9},
  {1, 17, 7, 32, 17, 10},
  {1, 18, 7, 32, 17, 12},
  {1, 19, 7, 31, 17, 13},
  {1, 20, 7, 30, 17, 14},
  {1, 21, 7, 30, 17, 15},
  {1, 22, 7, 29, 17, 16},
  {1, 23, 7, 29, 17, 17},
  {1, 24, 7, 28, 17, 19},
  {1, 25, 7, 27, 17, 20},
  {1, 26, 7, 26, 17, 21},
  {1, 27, 7, 25, 17, 22},
  {1, 28, 7, 25, 17, 24},
  {1, 29, 7, 24, 17, 25},
  {1, 30, 7, 23, 17, 26},
  {1, 31, 7, 22, 17, 27},
  {2, 1, 7, 21, 17, 28},
  {2, 2, 7, 20, 17, 30},
  {2, 3, 7, 19, 17, 31},
  {2, 4, 7, 18, 17, 32},
  {2, 5, 7, 17, 17, 33},
  {2, 6, 7, 16, 17, 35},
  {2, 7, 7, 15, 17, 36},
  {2, 8, 7, 14, 17, 37},
  {2, 9, 7, 12, 17, 38},
  {2, 10, 7, 11, 17, 40},
  {2, 11, 7, 10, 17, 41},
  {2, 12, 7, 9, 17, 42},
  {2, 13, 7, 8, 17, 43},
  {2, 14, 7, 6, 17, 45},
  {2, 15, 7, 5, 17, 46},
  {2, 16, 7, 4, 17, 47},
  {2, 17, 7, 2, 17, 48},
  {2, 18, 7, 1, 17, 49},
  {2, 19, 7, 0, 17, 51},
  {2, 20, 6, 58, 17, 52},
  {2, 21, 6, 57, 17, 53},
  {2, 22, 6, 55, 17, 54},
  {2, 23, 6, 54, 17, 55},
  {2, 24, 6, 53, 17, 56},
  {2, 25, 6, 51, 17, 58},
  {2, 26, 6, 50, 17, 59},
  {2, 27, 6, 48, 18, 0},
  {2, 28, 6, 47, 18, 1},
  {3, 1, 6, 45, 18, 2},
  {3, 2, 6, 44, 18, 3},
  {3, 3, 6, 42, 18, 4},
  {3, 4, 6, 40, 18, 6},
  {3, 5, 6, 39, 18, 7},
  {3, 6, 6, 37, 18, 8},
  {3, 7, 6, 36, 18, 9},
  {3, 8, 6, 34, 18, 10},
  {3, 9, 6, 32, 18, 11},
  {3, 10, 6, 31, 18, 12},
  {3, 11, 6, 29, 18, 13},
  {3, 12, 6, 28, 18, 15},
  {3, 13, 6, 26, 18, 16},
  {3, 14, 6, 24, 18, 17},
  {3, 15, 6, 23, 18, 18},
  {3, 16, 6, 21, 18, 19},
  {3, 17, 6, 19, 18, 20},
  {3, 18, 6, 18, 18, 21},
  {3, 19, 6, 16, 18, 22},
  {3, 20, 6, 14, 18, 23},
  {3, 21, 6, 13, 18, 24},
  {3, 22, 6, 11, 18, 25},
  {3, 23, 6, 9, 18, 26},
  {3, 24, 6, 8, 18, 27},
  {3, 25, 6, 6, 18, 28},
  {3, 26, 6, 4, 18, 30},
  {3, 27, 6, 3, 18, 31},
  {3, 28, 6, 1, 18, 32},
  {3, 29, 6, 0, 18, 33},
  {3, 30, 5, 58, 18, 34},
  {3, 31, 5, 56, 18, 35},
  {4, 1, 5, 55, 18, 36},
  {4, 2, 5, 53, 18, 37},
  {4, 3, 5, 51, 18, 38},
  {4, 4, 5, 50, 18, 39},
  {4, 5, 5, 48, 18, 40},
  {4, 6, 5, 46, 18, 41},
  {4, 7, 5, 45, 18, 42},
  {4, 8, 5, 43, 18, 43},
  {4, 9, 5, 42, 18, 44},
  {4, 10, 5, 40, 18, 45},
  {4, 11, 5, 38, 18, 46},
  {4, 12, 5, 37, 18, 47},
  {4, 13, 5, 35, 18, 48},
  {4, 14, 5, 34, 18, 50},
  {4, 15, 5, 32, 18, 51},
  {4, 16, 5, 31, 18, 52},
  {4, 17, 5, 29, 18, 53},
  {4, 18, 5, 28, 18, 54}
};
