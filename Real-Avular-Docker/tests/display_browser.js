// Run with agent-browser eval --stdin on the disabled dashboard.
// Tests presentation only, never robot commands.
(async()=>{
  const results=[];
  const check=(ok,label)=>{if(!ok)throw new Error(label);results.push(label);};
  if(mine||state?.enabled)throw new Error('Disable dashboard motion before UI tests');
  const oldConnection=connection,oldGrid=grid,oldImage=image,oldTheme=$('theme-select').value;
  const oldCompact=document.body.classList.contains('compact'),oldFocus=document.body.classList.contains('focus-map');
  const oldPanels=[...document.querySelectorAll('[data-panel]')].map(p=>[p,p.open]);
  const saved={...localStorage};
  const choose=value=>{$('theme-select').value=value;$('theme-select').dispatchEvent(new Event('change'));};
  try{
    check(!$('error').textContent,'No dashboard error on startup');
    const stopBackground=getComputedStyle($('stop')).backgroundImage;
    const safetyColors=['--pass','--fail','--warn'].map(name=>getComputedStyle(document.body).getPropertyValue(name).trim());
    connection={name:'Ronaldo',detected:true};choose('robot');
    check(document.body.dataset.accent==='ronaldo','Detected Ronaldo selects its automatic theme');
    for(const selector of ['.brand .logo','button.primary','h2::before']){
      const element=document.querySelector(selector==='h2::before'?'h2':selector);
      const gradient=getComputedStyle(element,selector==='h2::before'?'::before':null).backgroundImage;
      check(gradient.includes('rgb(248, 113, 113)')&&gradient.includes('rgb(74, 222, 128)'),selector+': Ronaldo red-green gradient');
    }
    const backdrop=getComputedStyle(document.body).backgroundImage;
    check(backdrop.includes('rgb(18, 53, 34)')&&backdrop.includes('rgb(50, 19, 25)'),'Ronaldo backdrop has green and red glow');
    check(getComputedStyle($('stop')).backgroundImage===stopBackground,'Ronaldo leaves STOP colors unchanged');
    connection={name:'Chud',detected:true,configured_name:'Ronaldo'};applyTheme();
    check(document.body.dataset.accent==='chud','Detected Chud overrides configured target');
    check(getComputedStyle(document.body).getPropertyValue('--accent').trim()==='#c49a78','Chud uses a brown accent');
    check(getComputedStyle(document.body).getPropertyValue('--panel').trim()==='#19120e','Chud uses brown panels');
    check(!getComputedStyle(document.querySelector('button.primary')).backgroundImage.includes('rgb(74, 222, 128)'),'Chud does not retain Ronaldo green');
    check(getComputedStyle($('stop')).backgroundImage===stopBackground,'Chud leaves STOP colors unchanged');
    check(['--pass','--fail','--warn'].every((name,i)=>getComputedStyle(document.body).getPropertyValue(name).trim()===safetyColors[i]),'Robot themes preserve safety indicator colors');
    connection={name:'Ronaldo',detected:false};applyTheme();
    check(!document.body.dataset.accent,'Undetected configured robot uses neutral accent');
    connection={name:'Unknown'};applyTheme();
    check(!document.body.dataset.accent,'Unknown robot uses neutral accent');
    grid={width:3,height:1,resolution:1,origin:[0,0,0],data:[-1,0,100]};
    for(const name of ['midnight','forest','violet','light']){
      choose(name);
      check(document.body.dataset.theme===name&&!document.body.dataset.accent,name+' theme applies');
      const rgb=image.getContext('2d').getImageData(0,0,3,1).data;
      check(rgb[0]!==rgb[8]||rgb[1]!==rgb[9],name+' map distinguishes unknown and walls');
      check(preferences.get('theme','')===name,name+' preference saved');
    }
    grid=oldGrid;image=oldImage;
    setDisplay('focus',false);$('focus-toggle').click();
    check(getComputedStyle($('flow')).display==='none'&&getComputedStyle($('robot-panel')).display==='none','Focus mode hides both side columns');
    check($('stop').getBoundingClientRect().width>0,'STOP stays visible in focus mode');
    $('focus-toggle').click();
    check(getComputedStyle($('flow')).display!=='none','Show panels restores workflow');
    setDisplay('compact',false);$('compact-toggle').click();
    check(document.body.classList.contains('compact')&&preferences.get('compact','')==='true','Compact toggles and persists');
    for(const [p] of oldPanels){
      p.open=false;p.dispatchEvent(new Event('toggle'));
      check(preferences.get('panel-'+p.dataset.panel,'')==='false',p.dataset.panel+' collapse persists');
      p.open=true;p.dispatchEvent(new Event('toggle'));
    }
    setDisplay('compact',false);choose('robot');
    const copy=document.documentElement.cloneNode(true);
    copy.querySelectorAll('script').forEach(s=>s.remove());
    copy.querySelector('body').classList.remove('focus-map','compact');
    const frame=document.createElement('iframe');
    frame.style.cssText='position:fixed;left:-10000px;top:0;border:0;height:1080px';
    document.body.append(frame);
    try{
      for(const width of [3840,1920,1280,900,420]){
        frame.style.width=width+'px';
        await new Promise(resolve=>{frame.onload=resolve;frame.srcdoc='<!doctype html>'+copy.outerHTML;});
        const d=frame.contentDocument,w=frame.contentWindow;
        check(d.documentElement.scrollWidth<=width,width+'px: no horizontal overflow');
        const main=d.querySelector('main').getBoundingClientRect();
        check(main.width>=d.documentElement.clientWidth-2,width+'px: full-width workspace');
        check(d.querySelector('#stop').getBoundingClientRect().width>0,width+'px: STOP visible');
        check(d.querySelector('#map').getBoundingClientRect().width>200,width+'px: usable map');
      }
    }finally{frame.remove();}
    return {passed:results.length,checks:results};
  }finally{
    connection=oldConnection;grid=oldGrid;image=oldImage;
    $('theme-select').value=oldTheme;applyTheme();
    setDisplay('compact',oldCompact);setDisplay('focus',oldFocus);
    for(const [p,open] of oldPanels){p.open=open;p.dispatchEvent(new Event('toggle'));}
    for(const key of Object.keys(localStorage))if(key.startsWith('coffe-ui-'))localStorage.removeItem(key);
    for(const [key,value] of Object.entries(saved))if(key.startsWith('coffe-ui-'))localStorage.setItem(key,value);
  }
})()
