#include <string.h>
#include <stdio.h>
#include "ntc.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
// #include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#define TAG     "adc"

/*
GPIO32  ADC1_CH4
GPIO33  ADC1_CH5
GPIO34  ADC1_CH6
GPIO35  ADC1_CH7
GPIO36  ADC1_CH0
GPIO37  ADC1_CH1
GPIO38  ADC1_CH2
GPIO39  ADC1_CH3
*/ 

// 这个我接GPIO3的话，查表是adc1
#define TEMP_ADC_CHANNEL     ADC_CHANNEL_1     //ADC转换通道(ADC1有8个通道，对应gpio32 - gpio39)

#define NTC_RES             9553               //NTC电阻标称值(在电路中和NTC一起串进电路的那个电阻,一般是10K，100K) 实测9916
#define ADC_V_MAX           3319                //最大接入电压值 实测3312

#define ADC_VALUE_NUM       10                  //每次采样的电压

static bool do_calibration1 = true;            //是否需要校准

static volatile float s_temp_value = 0.0f;      //室内温度



static int s_adc_raw[ADC_VALUE_NUM];              //ADC采样值
static int s_voltage_raw[ADC_VALUE_NUM];              //转换后的电压值


typedef struct
{
    int8_t temp;        //温度
    uint32_t res;       //温度对应的阻值
}temp_res_t;

//NTC温度表
static const temp_res_t s_ntc_table[] =
    {
    {-10, 44613.1},
    {-9, 42545.6},
    {-8, 40586.5},
    {-7, 38729.3},
    {-6, 36968.2},
    {-5, 35297.7},
    {-4, 33712.6},
    {-3, 32208.2},
    {-2, 30779.8},
    {-1, 29423.2},
    {0, 28134.4},
    {1, 26909.7},
    {2, 25745.4},
    {3, 24638.4},
    {4, 23585.4},
    {5, 22583.5},
    {6, 21630.1},
    {7, 20722.4},
    {8, 19858.1},
    {9, 19034.8},
    {10, 18250.3},
    {11, 17502.8},
    {12, 16790.1},
    {13, 16110.5},
    {14, 15462.3},
    {15, 14843.8},
    {16, 14253.6},
    {17, 13690.2},
    {18, 13152.3},
    {19, 12638.5},
    {20, 12147.7},
    {21, 11678.7},
    {22, 11230.4},
    {23, 10801.8},
    {24, 10392.0},
    {25, 10000.0},
    {26, 9625.0},
    {27, 9266.0},
    {28, 8922.5},
    {29, 8593.6},
    {30, 8278.6},
    {31, 7976.8},
    {32, 7687.8},
    {33, 7410.7},
    {34, 7145.2},
    {35, 6890.6},
    {36, 6646.4},
    {37, 6412.3},
    {38, 6187.6},
    {39, 5972.0},
    {40, 5765.1},
    {41, 5566.4},
    {42, 5375.7},
    {43, 5192.5},
    {44, 5016.5},
    {45, 4847.4},
    {46, 4685.0},
    {47, 4528.8},
    {48, 4378.6},
    {49, 4234.2},
    {50, 4095.3},
    {51, 3961.7},
    {52, 3833.2},
    {53, 3709.5},
    {54, 3590.4},
    {55, 3475.8},
    {56, 3365.4},
    {57, 3259.1},
    {58, 3156.7},
    {59, 3058.0},
    {60, 2962.9},
    {61, 2871.3},
    {62, 2782.9},
    {63, 2697.8},
    {64, 2615.6},
    {65, 2536.4},
    {66, 2460.0},
    {67, 2386.3},
    {68, 2315.1},
    {69, 2246.4},
    {70, 2180.1},
    {71, 2116.1},
    {72, 2054.3},
    {73, 1994.6},
    {74, 1937.0},
    {75, 1881.2},
    {76, 1827.4},
    {77, 1775.4},
    {78, 1725.1},
    {79, 1676.4},
    {80, 1629.4},
    {81, 1584.0},
    {82, 1540.0},
    {83, 1497.4},
    {84, 1456.3},
    {85, 1416.4},
    {86, 1377.9},
    {87, 1340.6},
    {88, 1304.4},
    {89, 1269.5},
    {90, 1235.6},
    {91, 1202.8},
    {92, 1171.0},
    {93, 1140.2},
    {94, 1110.3},
    {95, 1081.4},
    {96, 1053.4},
    {97, 1026.2},
    {98, 999.9},
    {99, 974.4},
    {100, 949.6},
    {101, 925.6},
    {102, 902.3},
    {103, 879.7},
    {104, 857.7},
    {105, 836.4},
    {106, 815.8},
    {107, 795.7},
    {108, 776.2},
    {109, 757.3},
    {110, 739.0},
    {111, 721.1},
    {112, 703.8},
    {113, 687.0},
    {114, 670.6},
    {115, 654.8},
    {116, 639.3},
    {117, 624.3},
    {118, 609.7},
    {119, 595.6},
    {120, 581.8},
    {121, 568.4},
    {122, 555.3},
    {123, 542.7},
    {124, 530.3},
    {125, 518.3}

};

//NTC表长度
static const uint16_t s_us_ntc_table_num = sizeof(s_ntc_table)/sizeof(s_ntc_table[0]);

//ADC操作句柄
static adc_oneshot_unit_handle_t s_adc_handle = NULL;

//转换句柄
static adc_cali_handle_t adc1_cali_handle = NULL;

//NTC温度采集任务
static void temp_adc_task(void*);

static float get_ntc_temp(uint32_t res);

/*---------------------------------------------------------------
        ADC校准方案，创建校准方案后，会从官方预烧录的参数中对采集到的电压进行校准
---------------------------------------------------------------*/
static bool example_adc_calibration_init(adc_unit_t unit, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

/**
 * 温度检测初始化
 * @param 无
 * @return 无
*/
void temp_ntc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,  //WIFI和ADC2无法同时启用，这里选择ADC1
    };

    //启用单次转换模式
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &s_adc_handle));

    //-------------ADC1 Config---------------//
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,       //分辨率
        .atten = ADC_ATTEN_DB_12,          
        //衰减倍数，ESP32设计的ADC参考电压为1100mV,只能测量0-1100mV之间的电压，如果要测量更大范围的电压
        //需要设置衰减倍数
        /*以下是对应可测量范围
        ADC_ATTEN_DB_0	    100 mV ~ 950 mV
        ADC_ATTEN_DB_2_5	100 mV ~ 1250 mV
        ADC_ATTEN_DB_6	    150 mV ~ 1750 mV
        ADC_ATTEN_DB_12	    150 mV ~ 2450 mV
        */
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, TEMP_ADC_CHANNEL, &config));
    //-------------ADC1 Calibration Init---------------//
    do_calibration1 = example_adc_calibration_init(ADC_UNIT_1, ADC_ATTEN_DB_12, &adc1_cali_handle);

    //新建一个任务，不断地进行ADC和温度计算
    xTaskCreatePinnedToCore(temp_adc_task, "adc_task", 2048, NULL,2, NULL, 0);
}

/**
 * 获取温度值
 * @param 无
 * @return 温度
*/
float get_temp(void)
{
    return s_temp_value;
}

static void temp_adc_task(void* param)
{
    uint16_t adc_cnt = 0;
    while(1)
    {
        adc_oneshot_read(s_adc_handle, TEMP_ADC_CHANNEL, &s_adc_raw[adc_cnt]);
        if (do_calibration1) {
            adc_cali_raw_to_voltage(adc1_cali_handle, s_adc_raw[adc_cnt], &s_voltage_raw[adc_cnt]);
        }
        adc_cnt++;
        if(adc_cnt >= 10)
        {
            int i = 0;
            //用平均值进行滤波
            uint32_t voltage = 0;
            uint32_t res = 0;
            for(i = 0;i < ADC_VALUE_NUM;i++)
            {
                voltage += s_voltage_raw[i];
            }
            voltage = voltage/ADC_VALUE_NUM;

            if(voltage < ADC_V_MAX)
            {
                //先接10k后接ntc的电压转换为相应的电阻值
                //res = (voltage*NTC_RES)/(ADC_V_MAX-voltage);
                //SEE先接ntc后接10k，测的ntc电压-dhb
                res = NTC_RES*(ADC_V_MAX-voltage)/(voltage);
                ESP_LOGI("ADCget","Original Value:%d",res);
                //根据电阻值查表找出对应的温度
                s_temp_value = get_ntc_temp(res);
            }

            adc_cnt = 0;
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

/** 线性插值，根据一元一次方程两点式，计算出K和B值，然后将X代入方程中计算出Y值
 * 所谓的线性插值，就是已知两点的坐标，由于两点坐标比较接近，将这两点之间的连线认为是一条直线
 * 然后通过这两点计算出这条直线的k和b值，
 * 插值的意思是，有一个x值，处于x1和x2之间，想要求出对应的y值
 * 这时候可以通过刚才计算出直线方程，计算出y值
 * @param x 需要计算的X坐标
 * @param x1,x2,y1,y2 两点式坐标
 * @return y值
*/
static float linera_interpolation(int32_t x,int32_t x1, int32_t x2, int32_t y1, int32_t y2)
{
    float k = (float)(y2 - y1) /(float)(x2 - x1);
    float b = (float)(x2 * y1 - x1 * y2) / (float)(x2 - x1);
    float y = k * x + b;
    return y;
}

/** 二分查找，通过电阻值查找出温度值对应的下标
 * @param res 电阻值
 * @param left ntc表的左边界
 * @param right ntc表的右边界
 * @return 温度值数组下标
*/
static int find_ntc_index(uint32_t  res,uint16_t left, uint16_t right)
{
    uint16_t middle = (left + right) / 2;
    if (right <= left || left == middle)
    {
        if (res >= s_ntc_table[left].res)
            return left;
        else
            return right;
    }
    if (res > s_ntc_table[middle].res)
    {
        right = middle;
        return find_ntc_index(res,left, right);
    }
    else if (res < s_ntc_table[middle].res)
    {
        left = middle;
        return find_ntc_index(res,left, right);
    }
    else
        return middle;
}

//根据电阻值、NTC表查出对应温度
static float get_ntc_temp(uint32_t res)
{
    uint16_t left = 0;
    uint16_t right = s_us_ntc_table_num - 1;
    int index = find_ntc_index(res,left,right);
    if (res == s_ntc_table[index].res)
        return s_ntc_table[index].temp;
    if (index == 0 || index == s_us_ntc_table_num - 1)
    {
        return s_ntc_table[index].temp;
    }
    else
    {
        int next_index = index + 1;
        return linera_interpolation(res, s_ntc_table[index].res, s_ntc_table[next_index].res, 
        s_ntc_table[index].temp, s_ntc_table[next_index].temp);
    }
}
