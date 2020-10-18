import xml.etree.ElementTree as Et
from xml.etree.ElementTree import Element
from xml.etree.ElementTree import SubElement
from pathlib import Path


def set_puck(raw_xml_path, xml_path, puck_size=None, puck_pos=None):

    puck_size = [0.03, 0.014] if puck_size is None else puck_size
    puck_pos = [0.3, 0, 0.014] if puck_pos is None else puck_pos

    with open(raw_xml_path, "r") as f:
        raw_xml = f.read()
        raw_xml = raw_xml.replace("[puck_size]", ' '.join(map(str, puck_size)))
        raw_xml = raw_xml.replace("[puck_pos]", ' '.join(map(str, puck_pos)))
        with open(xml_path, "w") as f1:
            f1.write(raw_xml)

