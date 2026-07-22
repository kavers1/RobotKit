 typedef struct __attribute__((packed))
{
- [x]    uint8_t  version[3];            /*@0    firmware version [major, minor, patch] — READ-ONLY */
- [x]    PWR_status_t  powersupply;      /*@3    power supply status. ie fixed 5V suppy or USB supply */ 
- [x]						uint8_t fixed : 1;
- [x]						uint8_t usb : 1;
	    					uint8_t padding : 4;
- [x]						uint8_t enable : 1 ;
- [x]						uint8_t reboot : 1 ;
- [x]    uint8_t  numberpdo;             /*@4    number of fixed voltage power delivery */
- [x]    uint8_t  numberpps;             /*@5    number of variable power delivey */
- [x]    uint16_t minvolt;               /*@6    minimal voltage of selected power delivery */
- [x]    uint16_t maxvolt;               /*@8    maximal voltage of selected power delivery */
- [x]    uint16_t maxcurrent;            /*@10   maximal current of selected power delivery */
- [x]    uint8_t  selected;              /*@12   selector of power delivery*/
- [x]    uint8_t  active;                /*@13   the currently active power delivery, most significant bit enables power to DRV's */
- [x]    uint16_t voltage;               /*@14   current voltage setpoint */
- [ ]    uint16_t current;               /*@16   current current setpoint */
- [X]	   drive_status_t  drv1status;     /*@18   fault status of DRV1 */
- [X]						uint8_t fault :1;   // value of the fault pin when not in address mode
	    					uint8_t reserved :7;
- [X]	   drive_config_t  drv1config;     /*@19   configuration of DRV1 */
- [X]						uint8_t sleep :1;
- [X]						uint8_t set_address :1;   // set fault to write address
		    				uint8_t reserved :5;
- [X]						uint8_t address_mode :1; // set fault pin to output to configure the I2C address of the DRV
- [X]    drive_control_t drv1control1;   /*@20   control1 register of DRV1 */
- [X]						uint8_t forward :1; 
- [X]						uint8_t reverse :1;
- [X]						uint8_t brake :1;
- [X]						uint8_t coast :1;
			    			uint8_t reserved :4;
- [X]    drive_control_t drv1control2;   /*@21   control2 register of DRV1 */
- [X]						uint8_t forward :1; 
- [X]						uint8_t reverse :1;
- [X]						uint8_t brake :1;
- [X]						uint8_t coast :1;
				    		uint8_t reserved :;
- [X]    int16_t  drv1speed1;            /*@22   speed1 setpoint of DRV1 */
- [X]    int16_t  drv1speed2;            /*@24   speed2 setpoint of DRV1 */
- [X]    drive_status_t  drv2status;     /*@26   fault status of DRV2 */
- [X]						uint8_t fault :1;   // value of the fault pin when not in address mode
					    	uint8_t reserved :7;
- [X]    drive_config_t  drv2config;     /*@27   configuration of DRV2 */
- [X]						uint8_t sleep :1;
- [X]						uint8_t set_address :1;   // set fault to write address
						    uint8_t reserved :5;
- [X]						uint8_t address_mode :1; // set fault pin to output to configure the I2C address of the DRV
- [X]	    drive_control_t drv2control3;   /*@28   control1 register of DRV2 */
- [X]						uint8_t forward :1; 
- [X]						uint8_t reverse :1;
- [X]						uint8_t brake :1;
- [X]						uint8_t coast :1;
						    uint8_t reserved :4;
- [X]    drive_control_t drv2control4;   /*@29   control2 register of DRV2 */
- [X]						uint8_t forward :1; // READ
- [X]						uint8_t reverse :1; // READ
- [X]						uint8_t brake :1;
- [X]						uint8_t coast :1;
						    uint8_t reserved :4;
- [X]    int16_t  drv2speed3;            /*@30   speed1 setpoint of DRV2*/
- [X]    int16_t  drv2speed4;            /*@32   speed2 setpoint of DRV2*/
}k robotkit_data_t;