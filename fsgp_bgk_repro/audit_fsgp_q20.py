"""Check saved predictions against their raw evidence and the original mapper."""
import hashlib
import json
import numpy as np
from scipy.spatial import cKDTree
from .compare_fsgp_q20 import OUT, METHODS, source, write_json
from .core import Config, TraversabilityMapper


def phi(xy):
    x,y=xy.T
    return np.column_stack((np.ones(len(x)),x,y,x*x,x*y,y*y))


def main():
    records=json.loads((OUT/'results.json').read_text())['cases']
    hill,_=source('hill')
    maxima=dict(q_height_m=0.,q_gradient=0.,q_variance_m2=0.,ols_prediction_m=0.,ols_valid_prediction_m=0.,ridge_prediction_m=0.,
                original_mapper_height_m=0.,original_mapper_variance_m2=0.,original_mapper_cost=0.)
    models=0;normals=0
    for case in records:
        with np.load(OUT/'cases'/(case['id']+'.npz')) as saved:
            points=saved['points'];xy=saved['query_xy'];seeds=saved['q_seeds']
            coeff=saved['q_coefficients'];cov=saved['q_covariance'];support=saved['q_support'];model_height=saved['q_height']
            known=saved['Q20__known'];ids=np.flatnonzero(known);s=support[known]
            assert np.array_equal(known,support>=0)
            local=xy[known]-seeds[s,:2];a=phi(local)
            height=seeds[s,2]+np.sum(a*coeff[s],axis=1)
            grad=np.column_stack((coeff[s,1]+2*coeff[s,3]*local[:,0]+coeff[s,4]*local[:,1],
                coeff[s,2]+coeff[s,4]*local[:,0]+2*coeff[s,5]*local[:,1]))
            var=np.einsum('ni,nij,nj->n',a,cov[s],a)
            maxima['q_height_m']=max(maxima['q_height_m'],float(np.max(np.abs(height-saved['Q20__height'][known]))))
            maxima['q_gradient']=max(maxima['q_gradient'],float(np.max(np.abs(grad-saved['Q20__gradient'][known]))))
            maxima['q_variance_m2']=max(maxima['q_variance_m2'],float(np.max(np.abs(var-saved['Q20__variance'][known]))))
            assert np.all(np.linalg.eigvalsh(cov[np.unique(s)])>=-1e-10)
            offsets=saved['q_neighbor_offsets'];pool_ids=saved['q_neighbor_ids']
            solved=np.flatnonzero(np.isfinite(coeff).all(axis=1))
            sample=solved[np.linspace(0,len(solved)-1,min(16,len(solved))).astype(int)]
            for i in sample:
                pool=pool_ids[offsets[i]:offsets[i+1]];delta=points[pool].astype(float)-seeds[i]
                assert len(pool)>=30
                assert np.all(np.abs(delta)<=np.array([.1,.1,.04])*(1+1.1e-7))
                independent,*_=np.linalg.lstsq(phi(delta[:,:2]),delta[:,2],rcond=None)
                difference=np.abs(phi(delta[:,:2])@(independent-coeff[i]))
                maxima['ols_prediction_m']=max(maxima['ols_prediction_m'],float(difference.max()))
                if np.isfinite(model_height[i]):
                    maxima['ols_valid_prediction_m']=max(maxima['ols_valid_prediction_m'],float(difference.max()))
                scale=np.sqrt(np.mean(np.sum(delta[:,:2]**2,axis=1)))
                design=phi(delta[:,:2]/scale);gram=design.T@design
                theta=np.linalg.solve(gram+1e-10*np.trace(gram)/6*np.eye(6),design.T@delta[:,2])
                physical=theta/np.array([1,scale,scale,scale**2,scale**2,scale**2])
                maxima['ridge_prediction_m']=max(maxima['ridge_prediction_m'],float(np.abs(phi(delta[:,:2])@(physical-coeff[i])).max()))
                models+=1
            common=saved['height_mask'].copy();risk=saved['risk_mask'].copy()
            for name in METHODS:
                common &= saved[name+'__known'];risk &= np.isfinite(saved[name+'__body_rank'])
                # Independently enumerate the discrete footprint instead of filtering.
                cost=saved[name+'__cost'].reshape(25,25);actual=saved[name+'__body_rank'].reshape(25,25)
                for y in range(25):
                    for x in range(25):
                        values=cost[max(0,y-1):min(25,y+2),max(0,x-2):min(25,x+3)]
                        finite=values[np.isfinite(values)]
                        expected=np.max(finite) if len(finite)>=12 else np.nan
                        assert (np.isnan(expected) and np.isnan(actual[y,x])) or expected==actual[y,x]
                if name.startswith('FSGP'):
                    retained=saved[name+'__inducing'][:,:3]
                    assert np.max(cKDTree(points).query(retained)[0])==0
            assert np.array_equal(common,saved['common'])
            assert np.array_equal(risk,saved['common_risk'])
            if case['scene']=='hill':
                np.testing.assert_array_equal(xy,hill[saved['test_ids'],:2])
                np.testing.assert_array_equal(saved['truth_height'],hill[saved['test_ids'],2])
                assert np.intersect1d(saved['source_ids'],saved['test_ids']).size==0
            if case['id'] in ('flat_dense_noisy_1102','corridor-040_dense_low_1101'):
                for count in (125,500):
                    result=TraversabilityMapper(Config(seed=case['seed'],inducing_points=count)).assess(points)
                    for key,field in [('height','original_mapper_height_m'),('variance','original_mapper_variance_m2'),('cost','original_mapper_cost')]:
                        before=saved[f'FSGP-{count}__{key}'];after=result[key].ravel()
                        np.testing.assert_allclose(before,after,atol=1e-12,rtol=1e-12,equal_nan=True)
                        maxima[field]=max(maxima[field],float(np.nanmax(np.abs(before-after))))
                    np.testing.assert_array_equal(saved[f'FSGP-{count}__known'],result['observed'])
                    normals+=1
    assert maxima['q_height_m']<1e-12 and maxima['q_gradient']<1e-12
    assert maxima['q_variance_m2']<1e-12 and maxima['ols_valid_prediction_m']<1e-6 and maxima['ridge_prediction_m']<1e-6
    write_json(OUT/'numerical-audit.json',dict(passed=True,cases=len(records),sampled_models=models,
        original_mapper_full_case_checks=normals,max_absolute_difference=maxima,
        checks=['all saved Q20 supported-query heights/derivatives/variances reconstructed from coefficients',
            'sampled raw pools satisfy the 20 cm/4 cm gates and match independent unregularised least squares',
            'retained GP observations exist exactly in the shared raw scan',
            'hill targets verified against official source indices, disjoint from training',
            'all discrete body-window maxima and common-support masks independently enumerated',
            'two complete scans times two inducing budgets checked against original TraversabilityMapper'],
        note='Numerical consistency audit, not real-robot or official-paper validation. OLS differs from the tiny-ridge solution; the largest sampled OLS discrepancy belongs to a rejected ill-conditioned fit. Accepted-fit OLS error and a direct normalised ridge solve are checked separately.'))
    print('Audited',len(records),'cases;',models,'OLS checks;',normals,'original mapper checks;',maxima)


if __name__=='__main__':main()
