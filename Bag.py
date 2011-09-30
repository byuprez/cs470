'''
A Bag is a generic data structure, especially useful for data that needs
to be serialized for network transmission or file storage. Bags have 
methods so that they can be used much like dictionaries as well.

This code is in the public domain.
'''

import re, json

class Bag(dict):
    def __repr__(self): return 'Bag' + super(Bag, self).__repr__()
    def __setattr__(self, k, v): self[k] = v
    def __getattr__(self, k):
        try: return self[k]
        except KeyError: raise AttributeError('No such attribute %r' % k)
    
    # Routines for conversion to/from JSON
    @staticmethod
    def FromJSON(s): 
        return _ForceBags(json.loads(s))

    def ToJSON(self, indent=None): 
        return json.dumps(_ConvertTypes(self), indent=indent)
    
def _ForceBags(obj):
    '''Converts any dictionaries to Bags'''
    if type(obj) is dict:
        b = Bag()
        for k,v in obj.items():
            b[str(k)] = _ForceBags(v)
        return b
    elif type(obj) in (list, tuple):
        return [_ForceBags(x) for x in obj]
    else:
        return obj
   
def _ConvertTypes(obj):
    '''Recursively inspects the objects to convert any date, time, or datetime objects to strings.'''
    import datetime
    from decimal import Decimal
    objType = type(obj)
    if objType in (list, tuple):
        return [_ConvertTypes(x) for x in obj]
    elif objType in (dict, Bag):
        d = {}
        for k,v in obj.items():
            d[k] = _ConvertTypes(v)
        return d
    elif objType in (datetime.date, datetime.time, datetime.datetime):
        return str(obj)
    elif objType is Decimal:
        return float(obj)
    elif hasattr(obj, '__dict__'):
        # Hmm... it appears to be an object of some sort - give it a try!
        d = {}
        for k,v in obj.__dict__.items():
            d[k] = _ConvertTypes(v)
        return d
    return obj
