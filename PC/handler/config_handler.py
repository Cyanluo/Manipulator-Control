import configparser
from os import path


class CfgInfo:
    def __init__(self):
        self.logConfigPath = path.join(path.dirname(path.abspath(__file__)), "../config/logger.conf")
        self.mqttConfigPath = path.join(path.dirname(path.abspath(__file__)), "../config/mqtt.conf")

    def getMqttCfg(self):
        cfgInfo = dict()

        conf = configparser.ConfigParser()
        conf.read(self.mqttConfigPath, encoding="utf-8")
        sections = conf.sections()

        for s in sections:
            cfgInfo[s] = dict(conf.items(s))

        return cfgInfo

    def getMqttAddress(self):
        return self.getMqttCfg()["address"]

    def getMqttPubTopic(self):
        return self.getMqttCfg()["topic-pub"]

    def getMqttSubTopic(self):
        return self.getMqttCfg()["topic-sub"]


if __name__ == '__main__':
    cfg = CfgInfo()
    print(cfg.getMqttAddress())
