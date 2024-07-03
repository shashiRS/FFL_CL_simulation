(()=>{"use strict";var e,r,n,t,o,i,a,d,c={},l={};function u(e){var r=l[e];if(void 0!==r){if(void 0!==r.error)throw r.error;return r.exports}var n=l[e]={id:e,exports:{}};try{var t={id:e,module:n,factory:c[e],require:u};u.i.forEach((function(e){e(t)})),n=t.module,t.factory.call(n.exports,n,n.exports,t.require)}catch(e){throw n.error=e,e}return n.exports}u.m=c,u.c=l,u.i=[],e=[],u.O=(r,n,t,o)=>{if(!n){var i=1/0;for(l=0;l<e.length;l++){for(var[n,t,o]=e[l],a=!0,d=0;d<n.length;d++)(!1&o||i>=o)&&Object.keys(u.O).every((e=>u.O[e](n[d])))?n.splice(d--,1):(a=!1,o<i&&(i=o));if(a){e.splice(l--,1);var c=t();void 0!==c&&(r=c)}}return r}o=o||0;for(var l=e.length;l>0&&e[l-1][2]>o;l--)e[l]=e[l-1];e[l]=[n,t,o]},u.n=e=>{var r=e&&e.__esModule?()=>e.default:()=>e;return u.d(r,{a:r}),r},u.d=(e,r)=>{for(var n in r)u.o(r,n)&&!u.o(e,n)&&Object.defineProperty(e,n,{enumerable:!0,get:r[n]})},u.hu=e=>e+"."+u.h()+".hot-update.js",u.miniCssF=e=>{},u.hmrF=()=>"runtime."+u.h()+".hot-update.json",u.h=()=>"4d6f560b6f00f8e2d525",u.g=function(){if("object"==typeof globalThis)return globalThis;try{return this||new Function("return this")()}catch(e){if("object"==typeof window)return window}}(),u.o=(e,r)=>Object.prototype.hasOwnProperty.call(e,r),r={},n="tsf_html_template:",u.l=(e,t,o,i)=>{if(r[e])r[e].push(t);else{var a,d;if(void 0!==o)for(var c=document.getElementsByTagName("script"),l=0;l<c.length;l++){var s=c[l];if(s.getAttribute("src")==e||s.getAttribute("data-webpack")==n+o){a=s;break}}a||(d=!0,(a=document.createElement("script")).charset="utf-8",a.timeout=120,u.nc&&a.setAttribute("nonce",u.nc),a.setAttribute("data-webpack",n+o),a.src=e),r[e]=[t];var f=(n,t)=>{a.onerror=a.onload=null,clearTimeout(p);var o=r[e];if(delete r[e],a.parentNode&&a.parentNode.removeChild(a),o&&o.forEach((e=>e(t))),n)return n(t)},p=setTimeout(f.bind(null,void 0,{type:"timeout",target:a}),12e4);a.onerror=f.bind(null,a.onerror),a.onload=f.bind(null,a.onload),d&&document.head.appendChild(a)}},u.r=e=>{"undefined"!=typeof Symbol&&Symbol.toStringTag&&Object.defineProperty(e,Symbol.toStringTag,{value:"Module"}),Object.defineProperty(e,"__esModule",{value:!0})},(()=>{var e,r,n,t,o={},i=u.c,a=[],d=[],c="idle";function l(e){c=e;for(var r=[],n=0;n<d.length;n++)r[n]=d[n].call(null,e);return Promise.all(r)}function s(e){if(0===r.length)return e();var n=r;return r=[],Promise.all(n).then((function(){return s(e)}))}function f(e){if("idle"!==c)throw new Error("check() is only allowed in idle status");return l("check").then(u.hmrM).then((function(t){return t?l("prepare").then((function(){var o=[];return r=[],n=[],Promise.all(Object.keys(u.hmrC).reduce((function(e,r){return u.hmrC[r](t.c,t.r,t.m,e,n,o),e}),[])).then((function(){return s((function(){return e?h(e):l("ready").then((function(){return o}))}))}))})):l(m()?"ready":"idle").then((function(){return null}))}))}function p(e){return"ready"!==c?Promise.resolve().then((function(){throw new Error("apply() is only allowed in ready status")})):h(e)}function h(e){e=e||{},m();var r=n.map((function(r){return r(e)}));n=void 0;var o=r.map((function(e){return e.error})).filter(Boolean);if(o.length>0)return l("abort").then((function(){throw o[0]}));var i=l("dispose");r.forEach((function(e){e.dispose&&e.dispose()}));var a,d=l("apply"),c=function(e){a||(a=e)},u=[];return r.forEach((function(e){if(e.apply){var r=e.apply(c);if(r)for(var n=0;n<r.length;n++)u.push(r[n])}})),Promise.all([i,d]).then((function(){return a?l("fail").then((function(){throw a})):t?h(e).then((function(e){return u.forEach((function(r){e.indexOf(r)<0&&e.push(r)})),e})):l("idle").then((function(){return u}))}))}function m(){if(t)return n||(n=[]),Object.keys(u.hmrI).forEach((function(e){t.forEach((function(r){u.hmrI[e](r,n)}))})),t=void 0,!0}u.hmrD=o,u.i.push((function(h){var m,v,y,g,b=h.module,_=function(n,t){var o=i[t];if(!o)return n;var d=function(r){if(o.hot.active){if(i[r]){var d=i[r].parents;-1===d.indexOf(t)&&d.push(t)}else a=[t],e=r;-1===o.children.indexOf(r)&&o.children.push(r)}else console.warn("[HMR] unexpected require("+r+") from disposed module "+t),a=[];return n(r)},u=function(e){return{configurable:!0,enumerable:!0,get:function(){return n[e]},set:function(r){n[e]=r}}};for(var f in n)Object.prototype.hasOwnProperty.call(n,f)&&"e"!==f&&Object.defineProperty(d,f,u(f));return d.e=function(e){return function(e){switch(c){case"ready":return l("prepare"),r.push(e),s((function(){return l("ready")})),e;case"prepare":return r.push(e),e;default:return e}}(n.e(e))},d}(h.require,h.id);b.hot=(m=h.id,v=b,g={_acceptedDependencies:{},_acceptedErrorHandlers:{},_declinedDependencies:{},_selfAccepted:!1,_selfDeclined:!1,_selfInvalidated:!1,_disposeHandlers:[],_main:y=e!==m,_requireSelf:function(){a=v.parents.slice(),e=y?void 0:m,u(m)},active:!0,accept:function(e,r,n){if(void 0===e)g._selfAccepted=!0;else if("function"==typeof e)g._selfAccepted=e;else if("object"==typeof e&&null!==e)for(var t=0;t<e.length;t++)g._acceptedDependencies[e[t]]=r||function(){},g._acceptedErrorHandlers[e[t]]=n;else g._acceptedDependencies[e]=r||function(){},g._acceptedErrorHandlers[e]=n},decline:function(e){if(void 0===e)g._selfDeclined=!0;else if("object"==typeof e&&null!==e)for(var r=0;r<e.length;r++)g._declinedDependencies[e[r]]=!0;else g._declinedDependencies[e]=!0},dispose:function(e){g._disposeHandlers.push(e)},addDisposeHandler:function(e){g._disposeHandlers.push(e)},removeDisposeHandler:function(e){var r=g._disposeHandlers.indexOf(e);r>=0&&g._disposeHandlers.splice(r,1)},invalidate:function(){switch(this._selfInvalidated=!0,c){case"idle":n=[],Object.keys(u.hmrI).forEach((function(e){u.hmrI[e](m,n)})),l("ready");break;case"ready":Object.keys(u.hmrI).forEach((function(e){u.hmrI[e](m,n)}));break;case"prepare":case"check":case"dispose":case"apply":(t=t||[]).push(m)}},check:f,apply:p,status:function(e){if(!e)return c;d.push(e)},addStatusHandler:function(e){d.push(e)},removeStatusHandler:function(e){var r=d.indexOf(e);r>=0&&d.splice(r,1)},data:o[m]},e=void 0,g),b.parents=a,b.children=[],a=[],h.require=_})),u.hmrC={},u.hmrI={}})(),(()=>{var e;u.g.importScripts&&(e=u.g.location+"");var r=u.g.document;if(!e&&r&&(r.currentScript&&(e=r.currentScript.src),!e)){var n=r.getElementsByTagName("script");n.length&&(e=n[n.length-1].src)}if(!e)throw new Error("Automatic publicPath is not supported in this browser");e=e.replace(/#.*$/,"").replace(/\?.*$/,"").replace(/\/[^\/]+$/,"/"),u.p=e+"../"})(),t=(e,r,n,t)=>{var o=document.createElement("link");return o.rel="stylesheet",o.type="text/css",o.onerror=o.onload=i=>{if(o.onerror=o.onload=null,"load"===i.type)n();else{var a=i&&("load"===i.type?"missing":i.type),d=i&&i.target&&i.target.href||r,c=new Error("Loading CSS chunk "+e+" failed.\n("+d+")");c.code="CSS_CHUNK_LOAD_FAILED",c.type=a,c.request=d,o.parentNode.removeChild(o),t(c)}},o.href=r,document.head.appendChild(o),o},o=(e,r)=>{for(var n=document.getElementsByTagName("link"),t=0;t<n.length;t++){var o=(a=n[t]).getAttribute("data-href")||a.getAttribute("href");if("stylesheet"===a.rel&&(o===e||o===r))return a}var i=document.getElementsByTagName("style");for(t=0;t<i.length;t++){var a;if((o=(a=i[t]).getAttribute("data-href"))===e||o===r)return a}},i=[],a=[],d=e=>({dispose:()=>{for(var e=0;e<i.length;e++){var r=i[e];r.parentNode&&r.parentNode.removeChild(r)}i.length=0},apply:()=>{for(var e=0;e<a.length;e++)a[e].rel="stylesheet";a.length=0}}),u.hmrC.miniCss=(e,r,n,c,l,s)=>{l.push(d),e.forEach((e=>{var r=u.miniCssF(e),n=u.p+r,d=o(r,n);d&&c.push(new Promise(((r,o)=>{var c=t(e,n,(()=>{c.as="style",c.rel="preload",r()}),o);i.push(d),a.push(c)})))}))},(()=>{u.b=document.baseURI||self.location.href;var e,r,n,t,o=u.hmrS_jsonp=u.hmrS_jsonp||{666:0},i={};function a(e){return new Promise(((r,n)=>{i[e]=r;var t=u.p+u.hu(e),o=new Error;u.l(t,(r=>{if(i[e]){i[e]=void 0;var t=r&&("load"===r.type?"missing":r.type),a=r&&r.target&&r.target.src;o.message="Loading hot update chunk "+e+" failed.\n("+t+": "+a+")",o.name="ChunkLoadError",o.type=t,o.request=a,n(o)}}))}))}function d(i){function a(e){for(var r=[e],n={},t=r.map((function(e){return{chain:[e],id:e}}));t.length>0;){var o=t.pop(),i=o.id,a=o.chain,c=u.c[i];if(c&&(!c.hot._selfAccepted||c.hot._selfInvalidated)){if(c.hot._selfDeclined)return{type:"self-declined",chain:a,moduleId:i};if(c.hot._main)return{type:"unaccepted",chain:a,moduleId:i};for(var l=0;l<c.parents.length;l++){var s=c.parents[l],f=u.c[s];if(f){if(f.hot._declinedDependencies[i])return{type:"declined",chain:a.concat([s]),moduleId:i,parentId:s};-1===r.indexOf(s)&&(f.hot._acceptedDependencies[i]?(n[s]||(n[s]=[]),d(n[s],[i])):(delete n[s],r.push(s),t.push({chain:a.concat([s]),id:s})))}}}}return{type:"accepted",moduleId:e,outdatedModules:r,outdatedDependencies:n}}function d(e,r){for(var n=0;n<r.length;n++){var t=r[n];-1===e.indexOf(t)&&e.push(t)}}u.f&&delete u.f.jsonpHmr,e=void 0;var c={},l=[],s={},f=function(e){console.warn("[HMR] unexpected require("+e.id+") to disposed module")};for(var p in r)if(u.o(r,p)){var h,m=r[p],v=!1,y=!1,g=!1,b="";switch((h=m?a(p):{type:"disposed",moduleId:p}).chain&&(b="\nUpdate propagation: "+h.chain.join(" -> ")),h.type){case"self-declined":i.onDeclined&&i.onDeclined(h),i.ignoreDeclined||(v=new Error("Aborted because of self decline: "+h.moduleId+b));break;case"declined":i.onDeclined&&i.onDeclined(h),i.ignoreDeclined||(v=new Error("Aborted because of declined dependency: "+h.moduleId+" in "+h.parentId+b));break;case"unaccepted":i.onUnaccepted&&i.onUnaccepted(h),i.ignoreUnaccepted||(v=new Error("Aborted because "+p+" is not accepted"+b));break;case"accepted":i.onAccepted&&i.onAccepted(h),y=!0;break;case"disposed":i.onDisposed&&i.onDisposed(h),g=!0;break;default:throw new Error("Unexception type "+h.type)}if(v)return{error:v};if(y)for(p in s[p]=m,d(l,h.outdatedModules),h.outdatedDependencies)u.o(h.outdatedDependencies,p)&&(c[p]||(c[p]=[]),d(c[p],h.outdatedDependencies[p]));g&&(d(l,[h.moduleId]),s[p]=f)}r=void 0;for(var _,E=[],w=0;w<l.length;w++){var I=l[w],k=u.c[I];k&&(k.hot._selfAccepted||k.hot._main)&&s[I]!==f&&!k.hot._selfInvalidated&&E.push({module:I,require:k.hot._requireSelf,errorHandler:k.hot._selfAccepted})}return{dispose:function(){var e;n.forEach((function(e){delete o[e]})),n=void 0;for(var r,t=l.slice();t.length>0;){var i=t.pop(),a=u.c[i];if(a){var d={},s=a.hot._disposeHandlers;for(w=0;w<s.length;w++)s[w].call(null,d);for(u.hmrD[i]=d,a.hot.active=!1,delete u.c[i],delete c[i],w=0;w<a.children.length;w++){var f=u.c[a.children[w]];f&&(e=f.parents.indexOf(i))>=0&&f.parents.splice(e,1)}}}for(var p in c)if(u.o(c,p)&&(a=u.c[p]))for(_=c[p],w=0;w<_.length;w++)r=_[w],(e=a.children.indexOf(r))>=0&&a.children.splice(e,1)},apply:function(e){for(var r in s)u.o(s,r)&&(u.m[r]=s[r]);for(var n=0;n<t.length;n++)t[n](u);for(var o in c)if(u.o(c,o)){var a=u.c[o];if(a){_=c[o];for(var d=[],f=[],p=[],h=0;h<_.length;h++){var m=_[h],v=a.hot._acceptedDependencies[m],y=a.hot._acceptedErrorHandlers[m];if(v){if(-1!==d.indexOf(v))continue;d.push(v),f.push(y),p.push(m)}}for(var g=0;g<d.length;g++)try{d[g].call(null,_)}catch(r){if("function"==typeof f[g])try{f[g](r,{moduleId:o,dependencyId:p[g]})}catch(n){i.onErrored&&i.onErrored({type:"accept-error-handler-errored",moduleId:o,dependencyId:p[g],error:n,originalError:r}),i.ignoreErrored||(e(n),e(r))}else i.onErrored&&i.onErrored({type:"accept-errored",moduleId:o,dependencyId:p[g],error:r}),i.ignoreErrored||e(r)}}}for(var b=0;b<E.length;b++){var w=E[b],I=w.module;try{w.require(I)}catch(r){if("function"==typeof w.errorHandler)try{w.errorHandler(r,{moduleId:I,module:u.c[I]})}catch(n){i.onErrored&&i.onErrored({type:"self-accept-error-handler-errored",moduleId:I,error:n,originalError:r}),i.ignoreErrored||(e(n),e(r))}else i.onErrored&&i.onErrored({type:"self-accept-errored",moduleId:I,error:r}),i.ignoreErrored||e(r)}}return l}}}self.webpackHotUpdatetsf_html_template=(e,n,o)=>{for(var a in n)u.o(n,a)&&(r[a]=n[a]);o&&t.push(o),i[e]&&(i[e](),i[e]=void 0)},u.hmrI.jsonp=function(e,o){r||(r={},t=[],n=[],o.push(d)),u.o(r,e)||(r[e]=u.m[e])},u.hmrC.jsonp=function(i,c,l,s,f,p){f.push(d),e={},n=c,r=l.reduce((function(e,r){return e[r]=!1,e}),{}),t=[],i.forEach((function(r){u.o(o,r)&&void 0!==o[r]&&(s.push(a(r)),e[r]=!0)})),u.f&&(u.f.jsonpHmr=function(r,n){e&&!u.o(e,r)&&u.o(o,r)&&void 0!==o[r]&&(n.push(a(r)),e[r]=!0)})},u.hmrM=()=>{if("undefined"==typeof fetch)throw new Error("No browser support: need fetch API");return fetch(u.p+u.hmrF()).then((e=>{if(404!==e.status){if(!e.ok)throw new Error("Failed to fetch update manifest "+e.statusText);return e.json()}}))},u.O.j=e=>0===o[e];var c=(e,r)=>{var n,t,[i,a,d]=r,c=0;if(i.some((e=>0!==o[e]))){for(n in a)u.o(a,n)&&(u.m[n]=a[n]);if(d)var l=d(u)}for(e&&e(r);c<i.length;c++)t=i[c],u.o(o,t)&&o[t]&&o[t][0](),o[i[c]]=0;return u.O(l)},l=self.webpackChunktsf_html_template=self.webpackChunktsf_html_template||[];l.forEach(c.bind(null,0)),l.push=c.bind(null,l.push.bind(l))})()})();