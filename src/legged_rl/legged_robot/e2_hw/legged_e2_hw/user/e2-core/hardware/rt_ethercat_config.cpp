//
// Created by han on 23-9-3.
//

#include "rt_ethercat_config.h"

#include "string"
#include "memory"
#include "Medulla.h"

//TODO: config
void rt_ethercat_config()
{
    slave_dict.clear();

    std::string name_1("first_mcu");
    std::shared_ptr<EthercatSlaveBase> tmp = std::make_shared<Medulla>(name_1, 1);
    slave_dict[0] = tmp;

    std::string name_2("second_mcu");
    std::shared_ptr<EthercatSlaveBase> tmp_2 = std::make_shared<Medulla>(name_2, 2);
    slave_dict[1] = tmp_2;

    std::string name_3("third_mcu");
    std::shared_ptr<EthercatSlaveBase> tmp_3 = std::make_shared<Medulla>(name_3, 3);
    slave_dict[2] = tmp_3;

    std::string name_4("forth_mcu");
    std::shared_ptr<EthercatSlaveBase> tmp_4 = std::make_shared<ImuRc>(name_4, 4);
    slave_dict[3] = tmp_4;
}
