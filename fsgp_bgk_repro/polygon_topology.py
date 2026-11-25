"""Small context-owned wrapper for the installed GEOS C polygon operations.

Uses the system GEOS library and its re-entrant API. Coordinates and areas are
in the caller's units; returned regions describe whole connected polygons.
"""
import ctypes as C
import ctypes.util
import numpy as np


class PolygonTopology:
    def __init__(self):
        library=ctypes.util.find_library('geos_c')
        if not library:raise RuntimeError('The system GEOS C library is required')
        self.lib=C.CDLL(library);self.owned=set();p=C.c_void_p;i=C.c_int;u=C.c_uint;d=C.c_double
        signatures={
            'GEOS_init_r':(p,[]),'GEOS_finish_r':(None,[p]),
            'GEOSCoordSeq_create_r':(p,[p,u,u]),'GEOSCoordSeq_setXY_r':(i,[p,p,u,d,d]),
            'GEOSCoordSeq_getXY_r':(i,[p,p,u,C.POINTER(d),C.POINTER(d)]),
            'GEOSCoordSeq_getSize_r':(i,[p,p,C.POINTER(u)]),
            'GEOSCoordSeq_destroy_r':(None,[p,p]),
            'GEOSGeom_createPointFromXY_r':(p,[p,d,d]),
            'GEOSGeom_createLinearRing_r':(p,[p,p]),
            'GEOSGeom_createPolygon_r':(p,[p,p,C.POINTER(p),u]),
            'GEOSGeom_createCollection_r':(p,[p,i,C.POINTER(p),u]),
            'GEOSGeom_destroy_r':(None,[p,p]),'GEOSGeomTypeId_r':(i,[p,p]),
            'GEOSGeom_clone_r':(p,[p,p]),
            'GEOSGetNumGeometries_r':(i,[p,p]),'GEOSGetGeometryN_r':(p,[p,p,i]),
            'GEOSGetNumInteriorRings_r':(i,[p,p]),'GEOSGetInteriorRingN_r':(p,[p,p,i]),
            'GEOSGetExteriorRing_r':(p,[p,p]),'GEOSGeom_getCoordSeq_r':(p,[p,p]),
            'GEOSUnaryUnion_r':(p,[p,p]),'GEOSDifference_r':(p,[p,p,p]),
            'GEOSConstrainedDelaunayTriangulation_r':(p,[p,p]),
            'GEOSArea_r':(i,[p,p,C.POINTER(d)]),
            'GEOSDistance_r':(i,[p,p,p,C.POINTER(d)]),
            'GEOSNearestPoints_r':(p,[p,p,p]),
        }
        for name,(result,args) in signatures.items():
            f=getattr(self.lib,name);f.restype=result;f.argtypes=args
        self.context=self.lib.GEOS_init_r()
        if not self.context:raise RuntimeError('Cannot initialize GEOS')

    def __enter__(self):return self
    def __exit__(self,*args):self.close()

    def close(self):
        for geometry in list(self.owned):self.destroy(geometry)
        if self.context:self.lib.GEOS_finish_r(self.context);self.context=None

    def keep(self,geometry):
        if not geometry:raise RuntimeError('GEOS polygon operation failed')
        self.owned.add(geometry);return geometry

    def destroy(self,geometry):
        if geometry in self.owned:
            self.lib.GEOSGeom_destroy_r(self.context,geometry);self.owned.remove(geometry)

    def ring(self,xy):
        xy=np.asarray(xy,float)
        if np.array_equal(xy[0],xy[-1]):xy=xy[:-1]
        if len(xy)<3:raise ValueError('Polygon needs at least three coordinates')
        xy=np.r_[xy,xy[:1]];seq=self.lib.GEOSCoordSeq_create_r(self.context,len(xy),2)
        for j,(x,y) in enumerate(xy):
            if not self.lib.GEOSCoordSeq_setXY_r(self.context,seq,j,x,y):raise RuntimeError('Invalid polygon coordinate')
        return self.keep(self.lib.GEOSGeom_createLinearRing_r(self.context,seq))

    def polygon(self,xy,holes=()):
        shell=self.ring(xy);rings=[self.ring(h) for h in holes]
        array=(C.c_void_p*len(rings))(*rings)
        polygon=self.keep(self.lib.GEOSGeom_createPolygon_r(self.context,shell,array,len(rings)))
        self.owned.remove(shell)
        for ring in rings:self.owned.remove(ring)
        return polygon

    def union(self,polygons):
        children=[self.polygon(p) for p in polygons]
        return self._union_children(children)

    def union_with(self,geometry,polygons):
        children=[self.keep(self.lib.GEOSGeom_clone_r(self.context,geometry))]+[
            self.polygon(p['exterior'],p.get('holes',())) if isinstance(p,dict) else self.polygon(p)
            for p in polygons]
        return self._union_children(children)

    def _union_children(self,children):
        array=(C.c_void_p*len(children))(*children)
        collection=self.keep(self.lib.GEOSGeom_createCollection_r(self.context,7,array,len(children)))
        for child in children:self.owned.remove(child)
        result=self.keep(self.lib.GEOSUnaryUnion_r(self.context,collection));self.destroy(collection)
        return result

    def area(self,geometry):
        result=C.c_double()
        if not self.lib.GEOSArea_r(self.context,geometry,C.byref(result)):raise RuntimeError('Cannot measure polygon')
        return result.value

    def point(self,xy):
        return self.keep(self.lib.GEOSGeom_createPointFromXY_r(self.context,*map(float,xy)))

    def distance(self,a,b):
        result=C.c_double()
        if not self.lib.GEOSDistance_r(self.context,a,b,C.byref(result)):raise RuntimeError('Cannot measure geometry distance')
        return result.value

    def nearest(self,a,b):
        seq=self.lib.GEOSNearestPoints_r(self.context,a,b)
        if not seq:raise RuntimeError('Cannot find nearest geometry points')
        try:
            result=[]
            for j in (0,1):
                x,y=C.c_double(),C.c_double()
                if not self.lib.GEOSCoordSeq_getXY_r(self.context,seq,j,C.byref(x),C.byref(y)):raise RuntimeError('Cannot read nearest point')
                result.append((x.value,y.value))
            return np.asarray(result)
        finally:self.lib.GEOSCoordSeq_destroy_r(self.context,seq)

    def coordinates(self,geometry):
        seq=self.lib.GEOSGeom_getCoordSeq_r(self.context,geometry);size=C.c_uint()
        if not seq or not self.lib.GEOSCoordSeq_getSize_r(self.context,seq,C.byref(size)):raise RuntimeError('Cannot read coordinates')
        coordinates=np.empty((size.value,2))
        for j in range(size.value):
            x,y=C.c_double(),C.c_double()
            if not self.lib.GEOSCoordSeq_getXY_r(self.context,seq,j,C.byref(x),C.byref(y)):raise RuntimeError('Cannot read coordinate')
            coordinates[j]=x.value,y.value
        return coordinates

    def polygons(self,geometry):
        kind=self.lib.GEOSGeomTypeId_r(self.context,geometry)
        if kind==3:
            exterior=self.lib.GEOSGetExteriorRing_r(self.context,geometry)
            if exterior:
                holes=[self.coordinates(self.lib.GEOSGetInteriorRingN_r(self.context,geometry,j))
                       for j in range(self.lib.GEOSGetNumInteriorRings_r(self.context,geometry))]
                yield dict(exterior=self.coordinates(exterior),holes=holes,area=self.area(geometry))
        elif kind in (6,7):
            for j in range(self.lib.GEOSGetNumGeometries_r(self.context,geometry)):
                yield from self.polygons(self.lib.GEOSGetGeometryN_r(self.context,geometry,j))

    def enclosed_gaps(self,geometry):
        result=[];parts=list(self.polygons(geometry))
        low=np.array([p['exterior'].min(0) for p in parts]);high=np.array([p['exterior'].max(0) for p in parts])
        for index,part in enumerate(parts):
            for ring in part['holes']:
                temporary=self.polygon(ring)
                possible=np.flatnonzero(np.all(low>=ring.min(0)-1e-12,axis=1)&np.all(high<=ring.max(0)+1e-12,axis=1))
                islands=[parts[j]['exterior'] for j in possible if j!=index]
                if islands:
                    # Subtract solid island exteriors. Any holes inside an
                    # island are visited separately, avoiding double counts.
                    island=self.union(islands)
                    gap=self.keep(self.lib.GEOSDifference_r(self.context,temporary,island));self.destroy(island)
                    result.extend(self.polygons(gap));self.destroy(gap)
                else:result.append(dict(exterior=ring,holes=[],area=self.area(temporary)))
                self.destroy(temporary)
        return result

    def triangles(self,region):
        polygon=self.polygon(region['exterior'],region.get('holes',()))
        triangles=self.keep(self.lib.GEOSConstrainedDelaunayTriangulation_r(self.context,polygon))
        result=[p['exterior'][:3] for p in self.polygons(triangles)]
        self.destroy(triangles);self.destroy(polygon)
        return np.asarray(result,float).reshape(-1,3,2)
