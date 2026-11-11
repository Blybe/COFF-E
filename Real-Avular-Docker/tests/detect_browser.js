// agent-browser eval --stdin, on a disabled dashboard. No robot commands.
(async()=>{
  if(mine||state?.enabled)throw new Error('Disable motion before UI tests');
  const oldApi=api,oldConnection=connection,oldTheme=$('theme-select').value;
  const calls=[],checks=[];
  let detected;
  const check=(ok,label)=>{if(!ok)throw new Error(label);checks.push(label);};
  api=async(path,body)=>{
    calls.push({path,body});
    if(body)throw new Error('Unexpected command during detection');
    if(path==='connection'){
      if(detected instanceof Error)throw detected;
      return detected;
    }
    return oldApi(path);
  };
  try{
    $('theme-select').value='robot';
    for(const [name,suffix,adapter] of [['Chud','RK5','WiFi'],['Ronaldo','CR7','WiFi 2'],['Chud','RK5','WiFi']]){
      detected={name,detected:true,ip:'192.168.192.1',default_map:{id:suffix,name:'Test map'},
        host_wifi:{ssid:'Origin PB4U9'+suffix,adapter,signal:84,known_robot:name},
        host_status:'Laptop Wi-Fi monitor live'};
      await $('detect-robot').onclick();
      check($('robot-name').textContent===name,name+': name updated');
      check(document.body.dataset.accent===name.toLowerCase(),name+': automatic theme updated');
      check($('load-chud').textContent.includes('Load '+name+' map'),name+': shortcut updated');
      check($('detect-result').textContent.includes(adapter)&&$('detect-result').textContent.includes(suffix),name+': Wi-Fi result visible');
      check(!$('detect-robot').disabled,name+': button re-enabled');
    }
    for(const status of ['Laptop Wi-Fi monitor stale','Laptop Wi-Fi monitor unavailable',
      'Multiple robot Wi-Fi adapters connected; disconnect one']){
      detected={name:'Unknown robot',detected:false,default_map:null,host_wifi:null,host_status:status};
      await $('detect-robot').onclick();
      check($('robot-name').textContent==='Unknown robot',status+': no configured identity fallback');
      check(!document.body.dataset.accent,status+': neutral automatic theme');
      check($('detect-result').textContent.includes(status),status+': explanation visible');
      check($('load-chud').textContent.includes('no robot detected'),status+': no old map shortcut');
    }
    detected=new Error('test connection failure');
    await $('detect-robot').onclick();
    check($('detect-result').textContent.includes('Detection failed'), 'Network failure explained');
    check(!$('detect-robot').disabled,'Network failure restores button');
    check(calls.every(c=>!c.body),'Detection issued GETs only');
    check(!mine&&!state.enabled,'Motion remains disabled');
    return {passed:checks.length,checks};
  }finally{
    api=oldApi;connection=oldConnection;$('theme-select').value=oldTheme;renderDetection();
    $('detect-result').textContent='Reads laptop Wi-Fi only. Does not enable motion.';
  }
})()
