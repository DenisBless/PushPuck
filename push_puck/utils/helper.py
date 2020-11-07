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


def set_target(raw_xml_path, xml_path, target_pos=None):
    target_pos = [1.2, 0, 0] if target_pos is None else target_pos
    with open(raw_xml_path, "r") as f:
        raw_xml = f.read()
        raw_xml = raw_xml.replace("[target_pos]", ' '.join(map(str, target_pos)))
        with open(xml_path, "w") as f1:
            f1.write(raw_xml)


def has_collision(obj1_name, obj2_name, sim):
    obj1 = sim.model.geom_name2id(obj1_name)
    obj2 = sim.model.geom_name2id(obj2_name)
    for i in range(sim.data.ncon):
        contact = sim.data.contact[i]
        if contact.geom1 == obj1 and contact.geom2 == obj2:
            return True
        elif contact.geom1 == obj2 and contact.geom2 == obj1:
            return True
    return False
