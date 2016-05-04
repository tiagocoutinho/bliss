from __future__ import absolute_import
import logging
from bliss.common import log

def create_objects_from_config_node(config, item_cfg_node):
    #import pdb; pdb.set_trace()
    parent_node = item_cfg_node.parent
    item_name = item_cfg_node['name']

    module = __import__('bliss.controllers.temperature.%s' % parent_node['class'], fromlist=[None])
    
    inputs = list()
    outputs = list()
    loops = list()
    names = dict()
    for category, objects in [('inputs', inputs),('outputs', outputs), ('ctrl_loops', loops)]:
      for config_item in parent_node.get(category):
          name = config_item.get("name")
          objects.append((name, config_item))
          names.setdefault(category, list()).append(name)
                 
    controller_class = getattr(module, parent_node["class"])
    controller = controller_class(parent_node, inputs, outputs, loops)
    
    cache_dict = dict()
    for category in ('inputs', 'outputs', 'ctrl_loops'):
        cache_dict.update(dict(zip(names[category], [controller]*len(names[category]))))

    #controller.initialize()
    o = controller.get_object(item_name)
    if item_name in dict(loops).keys():
        referenced_object = o.config['input'][1:]
        if referenced_object in controller._objects:
           # referencing an object in same controller
           o._Loop__input = controller._objects[referenced_object]
        else:
           o._Loop__input = config.get(referenced_object)
	referenced_object = o.config['output'][1:]
        if referenced_object in controller._objects:
           # referencing an object in same controller
           o._Loop__output = controller._objects[referenced_object]
        else:
           o._Loop__output = config.get(referenced_object)
	
    return { item_name: o}, cache_dict
    
def create_object_from_cache(config, name, controller):
    o = controller.get_object(name)
    return o