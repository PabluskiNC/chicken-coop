var Sequelize = require('sequelize')
 ,  yaml      = require('read-yaml')
 ,  path      = require('path')
 ,  config    = yaml.sync(path.resolve(__dirname, '../config.yml'))
 ,  dbOptions = {};

if (config.db.dialect == "sqlite") {
  dbOptions = {
    host: config.db.host,
    dialect: config.db.dialect,
    pool: config.db.pool,
    storage: config.db.storage,
    logging: false
  }
}
else {
  dbOptions = {
    host: config.db.host,
    dialect: config.db.dialect,
    pool: config.db.pool,
    logging: false
  }
}

var db = new Sequelize(config.db.db_name, config.db.user, config.db.pass, dbOptions);
console.log('Preparing database...');

var models = [
  'Temperature',
  'Brightness',
  'Status'
];

models.forEach(function(model) {
  module.exports[model] = db.import(__dirname + '/' + model);
  module.exports[model].sync(); // Ensure tables exist in db.
});

  // describe relationships
  // (function(m) {
  //   m.PhoneNumber.belongsTo(m.User);
  //   m.Task.belongsTo(m.User);
  //   m.User.hasMany(m.Task);
  //   m.User.hasMany(m.PhoneNumber);
  // })(module.exports);

module.exports.db = db;
